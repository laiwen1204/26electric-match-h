# vision-maixcam — 钢球视觉检测与图传 (MaixCAM Pro)

2026 电赛 H 题视觉端：检测摆杆凹槽内钢球位置，UART 实时上报给 STM32，同时 RTSP 图传回场外接收端（对应赛题要求1）。

## 功能

- **检测**：YOLO26 钢球检测模型（`yolo26_all.cvimodel`，自训练）
- **跟踪**：`ball_tracker.py` 鲁棒跟踪器——置信度迟滞（0.50 锁定 / 0.35 两帧确认）、运动门关联、丢帧惯性外推（最多 4 帧）、α-β 自适应滤波
- **坐标换算**：`ball_position.py` 将球心像素投影到摆杆轴线，按标定端点换算为物理坐标（-12.5~+12.5cm）
- **上报**：UART(115200) 每 2 帧发 `found,pos_cm\n`（found=1 跟踪中 / 0 丢失）
- **图传**：RTSP 推流（320×180），场外笔记本/PAD 拉流实时观看钢球轨迹；本机屏同步显示检测框与误差

## 文件

| 文件 | 说明 |
|---|---|
| `main.py` | 主程序：检测→跟踪→坐标换算→UART 上报→RTSP 推流 |
| `ball_tracker.py` | 鲁棒跟踪器（关联/滤波/丢帧外推） |
| `ball_position.py` | 像素→物理坐标几何换算与标定校验 |
| `yolo26_all.mud/.cvimodel` | YOLO26 模型（MaixCAM 格式） |
| `app.yaml` / `app.png` | MaixCAM 应用描述/图标 |

## 部署

将本目录拷贝到 MaixCAM Pro，通过 MaixPy IDE 运行 `main.py`；RTSP 地址见启动日志。
