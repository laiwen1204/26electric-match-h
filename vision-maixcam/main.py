from maix import app, camera, display, gpio, image, nn, pinmap, time, uart

from ball_position import position_from_pixel, validate_calibration
from ball_tracker import RobustBallTracker

# Optional hardware light. Keep this False when auxiliary lighting is forbidden.
USE_FLASH_LIGHT = False

# Keep the official camera path and do not force 60 FPS. This gives automatic
# exposure more room in indoor light when no auxiliary light is available.
LOW_LATENCY_MODE = True
USE_RTSP = True

# Detector hysteresis: retrieve weak candidates at 0.35, but only a candidate
# at or above 0.50 can lock immediately. Weak candidates need two frames.
DETECTION_CONFIDENCE = 0.35
ACQUIRE_CONFIDENCE = 0.50
NMS_IOU_THRESHOLD = 0.45
LOW_CONF_CONFIRM_FRAMES = 2
MAX_COAST_FRAMES = 4

# Serial format: found_flag,position_cm\n  (1 when tracked, 0 when lost)
UART_SEND_INTERVAL = 2
UART_SEND_PREDICTED = False
UART_SEND_LOST_MARKER = True

# Show only the requested target status, errors, distance, and target box.
SHOW_TARGET_INFO = True

AXIS_START_CM = -12.5
AXIS_END_CM = 12.5
MODEL_PATH = "yolo26_all.mud"


flash_light = None
if USE_FLASH_LIGHT:
    pinmap.set_pin_function("B3", "GPIO")
    flash_light = gpio.GPIO("B3", gpio.Mode.OUT)
    flash_light.value(1)

pinmap.set_pin_function("A16", "UART0_TX")
pinmap.set_pin_function("A17", "UART0_RX")
serial_dev = uart.UART("/dev/ttyS0", 115200)

detector = nn.YOLO26(
    model=MODEL_PATH,
    dual_buff=not LOW_LATENCY_MODE,
)
frame_width = detector.input_width()
frame_height = detector.input_height()

axis_start_px = (5, frame_height // 2)
axis_end_px = (frame_width - 5, frame_height // 2)
validate_calibration(
    axis_start_px,
    axis_end_px,
    AXIS_START_CM,
    AXIS_END_CM,
    frame_width,
    frame_height,
)

cam = camera.Camera(frame_width, frame_height, detector.input_format())
disp = display.Display()

if USE_RTSP:
    from maix import rtsp

    rtsp_channel = cam.add_channel(320, 180, image.Format.FMT_YVU420SP)
    rtsp_server = rtsp.Rtsp()
    rtsp_server.bind_camera(rtsp_channel)
    rtsp_server.start()
    print("RTSP URL: {}".format(rtsp_server.get_url()))

tracker = RobustBallTracker(
    frame_width,
    frame_height,
    acquire_confidence=ACQUIRE_CONFIDENCE,
    low_conf_confirm_frames=LOW_CONF_CONFIRM_FRAMES,
    max_coast_frames=MAX_COAST_FRAMES,
)

measured_color = image.Color.from_rgb(0, 255, 0)
predicted_color = image.Color.from_rgb(255, 200, 0)
lost_color = image.Color.from_rgb(255, 0, 0)


frame_index = 0
while not app.need_exit():
    img = cam.read()
    now_ms = time.ticks_ms()

    raw_objs = detector.detect(
        img,
        conf_th=DETECTION_CONFIDENCE,
        iou_th=NMS_IOU_THRESHOLD,
    )
    target_state = tracker.update(raw_objs, now_ms)

    send_due = (
        UART_SEND_INTERVAL <= 1
        or frame_index % UART_SEND_INTERVAL == 0
    )

    if target_state is not None:
        center_x, center_y, box_w, box_h, measured, _score, _miss_frames = target_state
        position_cm, _axis_ratio, _axis_distance = position_from_pixel(
            (center_x, center_y),
            axis_start_px,
            axis_end_px,
            AXIS_START_CM,
            AXIS_END_CM,
        )

        relative_x = center_x - frame_width // 2
        relative_y = frame_height // 2 - center_y
        if send_due and (measured or UART_SEND_PREDICTED):
            serial_dev.write_str(
                "1,{:.2f}\n".format(position_cm)
            )

        if SHOW_TARGET_INFO:
            color = measured_color if measured else predicted_color
            box_x = max(0, center_x - box_w // 2)
            box_y = max(0, center_y - box_h // 2)
            img.draw_rect(box_x, box_y, box_w, box_h, color=color, thickness=2)
            img.draw_string(
                8,
                28,
                "BALL:{}".format("YES" if measured else "PRED"),
                color=color,
                thickness=1,
            )
            img.draw_string(
                8,
                48,
                "ERR:({:+d},{:+d})".format(relative_x, relative_y),
                color=color,
                thickness=1,
            )
            img.draw_string(
                8,
                68,
                "DIST_ERR:{:+.2f}cm".format(position_cm),
                color=color,
                thickness=1,
            )
    else:
        if send_due and UART_SEND_LOST_MARKER:
            serial_dev.write_str("0,0.00\n")
        if SHOW_TARGET_INFO:
            img.draw_string(
                8,
                28,
                "BALL:NO",
                color=lost_color,
                thickness=1,
            )
            img.draw_string(8, 48, "ERR:(--,--)", color=lost_color, thickness=1)
            img.draw_string(
                8,
                68,
                "DIST_ERR:--cm",
                color=lost_color,
                thickness=1,
            )

    fps = time.fps()
    img.draw_string(
        8,
        8,
        "FPS:{:.1f}".format(fps),
        color=image.COLOR_GREEN,
        thickness=1,
    )

    disp.show(img)
    frame_index += 1
