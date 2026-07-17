import cv2
import numpy as np
import pyrealsense2 as rs

# 1. 填入您的相机内参 (这里以一组虚拟值做演示，请务必替换为您相机的真实内参！)
# 格式为: [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
camera_matrix = np.array([[611.69867, 0.0, 326.48343],
                          [0.0, 610.4816, 242.60501],
                          [0.0, 0.0, 1.0]], dtype=np.float32)

# 填入畸变系数 (通常为5个值，若标定过请填入，若没有则暂设为0)
dist_coeffs = np.zeros((5, 1), dtype=np.float32)

# 2. 定义您打印的 ArUco 标记的物理大小（单位：米）
# 比如打印出来的黑色正方形边长为 10 厘米
marker_size = 0.123  

# 定义 ArUco 标记在自身局部坐标系下的 4 个角点的 3D 物理坐标
# 原点定在正方形中心，Z 轴垂直正方形面向上
obj_points = np.array([
    [-marker_size/2.0,  marker_size/2.0, 0.0],
    [ marker_size/2.0,  marker_size/2.0, 0.0],
    [ marker_size/2.0, -marker_size/2.0, 0.0],
    [-marker_size/2.0, -marker_size/2.0, 0.0]
], dtype=np.float32)

# 3. 初始化 OpenCV ArUco 检测器 (适用于 OpenCV 4.7以下版本)
# 我们使用的是标准 DICT_6X6_250 字典，编号为 0
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters_create()

# 4. 开启 RealSense 相机并进行实时计算
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# 等待相机稳定
for _ in range(30):
    pipeline.wait_for_frames()
print("RealSense 相机已启动")

print("程序已运行，按 'S' 键捕获当前位置数据，按 'Q' 键退出。")

while True:
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    if not color_frame:
        print("无法读取相机画面")
        break
    frame = np.asanyarray(color_frame.get_data())
        
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # 检测画面中的标记
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
    # 如果检测到了标记
    if ids is not None and len(ids) > 0:
        # 绘制检测到的标记边框
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        
        # 只针对我们需要的 ID 0 标记进行姿态估计
        for idx, marker_id in enumerate(ids.flatten()):
            if marker_id == 0:
                # 获取该标记在图像上的 2D 像素角点坐标
                img_points = corners[idx][0]
                
                # 核心步骤：使用 solvePnP 算法估计标定板相对于相机镜头的 3D 位姿
                # 输出 rvec（旋转向量）和 tvec（平移向量）
                success, rvec, tvec = cv2.solvePnP(
                    obj_points, img_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
                
                if success:
                    # 在画面上绘制一个 3D 坐标轴（方便您用肉眼确认检测准不准）
                    # 红色为 X 轴，绿色为 Y 轴，蓝色为 Z 轴
                    cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.05)
                    
                    # 实时在控制台打印当前算出的 rvec 和 tvec
                    # tvec 的 [0], [1], [2] 就是物体在相机下的 X, Y, Z 三维坐标 (米)
                    print(f"实时位置 -> rvec: {rvec.flatten()}, tvec: {tvec.flatten()}")
                    
                    # 保存触发机制：当您在键盘上按下 's' 键时，保存这组数据
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('s') or key == ord('S'):
                        print("\n[捕获成功] 已为您锁定当前时刻的数据：")
                        print("标定板当前旋转向量 (rvec):", rvec.flatten())
                        print("标定板当前平移向量 (tvec):", tvec.flatten())
                        print("请立即去记录此时您的机械臂 SDK 读数，作为一组配对数据！\n")
    
    cv2.imshow("Hand-Eye Calibration Camera Feed", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

pipeline.stop()
cv2.destroyAllWindows()