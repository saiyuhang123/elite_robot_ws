from ultralytics import YOLO

# 加载你的模型 (比如 yolov8s.pt)
model = YOLO('yolov8s.pt')

# 导出为 TensorRT engine，开启 FP16 半精度加速
model.export(format='engine', half=True, dynamic=False)