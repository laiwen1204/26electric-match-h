"""Low-miss tracking helpers for the steel-ball detector."""


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


class RobustBallTracker:
    """Associate detections, smooth measurements, and bridge short misses.

    High-confidence detections lock immediately. Lower-confidence detections
    need two consistent frames only while acquiring. Once locked, candidates
    are selected with a broad motion gate instead of hard ROI/shape filters.
    """

    def __init__(
        self,
        frame_width,
        frame_height,
        acquire_confidence=0.50,
        low_conf_confirm_frames=2,
        max_coast_frames=4,
    ):
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.acquire_confidence = acquire_confidence
        self.low_conf_confirm_frames = low_conf_confirm_frames
        self.max_coast_frames = max_coast_frames

        self.base_x_gate = max(20.0, frame_width * 0.12)
        self.base_y_gate = max(14.0, frame_height * 0.18)
        self.max_x_gate = frame_width * 0.38
        self.max_y_gate = frame_height * 0.35
        self.miss_gate_growth_x = frame_width * 0.05
        self.miss_gate_growth_y = frame_height * 0.04
        self.pending_x_gate = max(18.0, frame_width * 0.10)
        self.pending_y_gate = max(12.0, frame_height * 0.15)
        self.max_speed_x = frame_width * 5.0
        self.max_speed_y = frame_height * 3.0
        self.reset()

    def reset(self):
        self.locked = False
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.box_w = 0.0
        self.box_h = 0.0
        self.last_ms = None
        self.last_score = 0.0
        self.miss_frames = 0
        self.pending = None
        self.pending_frames = 0

    @staticmethod
    def _measurement(obj):
        if obj.w <= 0 or obj.h <= 0:
            return None
        return (
            obj.x + obj.w * 0.5,
            obj.y + obj.h * 0.5,
            float(obj.w),
            float(obj.h),
            float(obj.score),
        )

    def _measurements(self, objs):
        result = []
        for obj in objs:
            measurement = self._measurement(obj)
            if measurement is not None:
                result.append(measurement)
        return result

    def _state(self, measured):
        return (
            int(self.x + 0.5),
            int(self.y + 0.5),
            max(1, int(self.box_w + 0.5)),
            max(1, int(self.box_h + 0.5)),
            measured,
            self.last_score,
            self.miss_frames,
        )

    def _start(self, measurement, now_ms):
        self.x, self.y, self.box_w, self.box_h, self.last_score = measurement
        self.vx = 0.0
        self.vy = 0.0
        self.last_ms = now_ms
        self.miss_frames = 0
        self.locked = True
        self.pending = None
        self.pending_frames = 0
        return self._state(True)

    def _acquire(self, measurements, now_ms):
        if not measurements:
            self.pending = None
            self.pending_frames = 0
            return None

        candidate = max(measurements, key=lambda item: item[4])
        if candidate[4] >= self.acquire_confidence:
            return self._start(candidate, now_ms)

        if self.pending is None:
            consistent = False
        else:
            consistent = (
                abs(candidate[0] - self.pending[0]) <= self.pending_x_gate
                and abs(candidate[1] - self.pending[1]) <= self.pending_y_gate
            )

        self.pending_frames = self.pending_frames + 1 if consistent else 1
        self.pending = candidate
        if self.pending_frames >= self.low_conf_confirm_frames:
            return self._start(candidate, now_ms)
        return None

    def _predict(self, now_ms):
        dt = (now_ms - self.last_ms) * 0.001
        dt = _clamp(dt, 0.005, 0.120)
        return self.x + self.vx * dt, self.y + self.vy * dt, dt

    def _find_associated(self, measurements, predicted_x, predicted_y, dt):
        x_gate = self.base_x_gate + abs(self.vx) * dt * 1.5
        y_gate = self.base_y_gate + abs(self.vy) * dt * 1.5
        x_gate += self.miss_frames * self.miss_gate_growth_x
        y_gate += self.miss_frames * self.miss_gate_growth_y
        x_gate = min(self.max_x_gate, x_gate)
        y_gate = min(self.max_y_gate, y_gate)

        best = None
        best_cost = 1000000.0
        old_size = (self.box_w + self.box_h) * 0.5
        for measurement in measurements:
            x, y, box_w, box_h, score = measurement
            dx = x - predicted_x
            dy = y - predicted_y
            if abs(dx) > x_gate or abs(dy) > y_gate:
                continue

            new_size = (box_w + box_h) * 0.5
            size_difference = abs(new_size - old_size) / max(1.0, new_size + old_size)
            position_cost = (dx / x_gate) ** 2 + (dy / y_gate) ** 2
            cost = position_cost + size_difference * 0.30 - score * 0.12
            if cost < best_cost:
                best = measurement
                best_cost = cost
        return best

    def _apply_measurement(self, measurement, predicted_x, predicted_y, dt, now_ms):
        measured_x, measured_y, box_w, box_h, score = measurement
        error_x = measured_x - predicted_x
        error_y = measured_y - predicted_y
        normalized_error = abs(error_x) / max(1.0, self.base_x_gate)
        motion = min(1.0, normalized_error) ** 2

        alpha_x = 0.45 + 0.43 * motion
        alpha_y = 0.32 + 0.35 * motion
        beta = 0.035 + 0.085 * motion

        self.x = predicted_x + alpha_x * error_x
        self.y = predicted_y + alpha_y * error_y
        self.vx += beta * error_x / dt
        self.vy += beta * error_y / dt
        self.vx = _clamp(self.vx, -self.max_speed_x, self.max_speed_x)
        self.vy = _clamp(self.vy, -self.max_speed_y, self.max_speed_y)
        self.box_w += 0.30 * (box_w - self.box_w)
        self.box_h += 0.30 * (box_h - self.box_h)
        self.x = _clamp(self.x, 0.0, self.frame_width - 1.0)
        self.y = _clamp(self.y, 0.0, self.frame_height - 1.0)
        self.last_ms = now_ms
        self.last_score = score
        self.miss_frames = 0
        return self._state(True)

    def _coast(self, predicted_x, predicted_y, now_ms):
        self.miss_frames += 1
        if self.miss_frames > self.max_coast_frames:
            self.reset()
            return None

        self.x = _clamp(predicted_x, 0.0, self.frame_width - 1.0)
        self.y = _clamp(predicted_y, 0.0, self.frame_height - 1.0)
        self.vx *= 0.86
        self.vy *= 0.80
        self.last_ms = now_ms
        return self._state(False)

    def update(self, objs, now_ms):
        measurements = self._measurements(objs)
        if not self.locked:
            return self._acquire(measurements, now_ms)

        predicted_x, predicted_y, dt = self._predict(now_ms)
        candidate = self._find_associated(
            measurements, predicted_x, predicted_y, dt
        )

        # After one coast frame, allow a strong detection anywhere to relock.
        if candidate is None and self.miss_frames >= 1 and measurements:
            global_candidate = max(measurements, key=lambda item: item[4])
            if global_candidate[4] >= self.acquire_confidence:
                return self._start(global_candidate, now_ms)

        if candidate is None:
            return self._coast(predicted_x, predicted_y, now_ms)

        return self._apply_measurement(
            candidate, predicted_x, predicted_y, dt, now_ms
        )
