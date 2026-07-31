from ultralytics import YOLO

model = YOLO("/home/nvidia/Documents/elite_robot_ws/YOLO/yolo11x.pt")

results = model.train(
    data="data.yaml",
    epochs=300,
    imgsz=640,
    batch=8,
    optimizer="AdamW",
    lr0=0.001,
    lrf=0.01,
    cos_lr=True,
    warmup_epochs=5,
    warmup_momentum=0.8,
    warmup_bias_lr=0.1,
    # 数据增强
    hsv_h=0.015,
    hsv_s=0.7,
    hsv_v=0.4,
    degrees=10.0,
    translate=0.1,
    scale=0.5,
    shear=2.0,
    perspective=0.0,
    flipud=0.0,
    fliplr=0.5,
    mosaic=1.0,
    mixup=0.1,
    copy_paste=0.1,
    # 其他
    close_mosaic=30,
    amp=False,
    patience=50,
    save_period=50,
    device=0,
    project="runs/detect",
    name="fruit_yolo11x",
)
