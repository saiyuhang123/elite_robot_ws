import pyrealsense2 as rs
import numpy as np

# 创建并启动 pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# 等待几帧让相机稳定
for _ in range(30):
    pipeline.wait_for_frames()

# 获取内参
profile = pipeline.get_active_profile()
color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
intrinsics = color_profile.get_intrinsics()
# 转换为 OpenCV 需要的矩阵格式
camera_matrix = np.array([[intrinsics.fx, 0, intrinsics.ppx], 
                          [0, intrinsics.fy, intrinsics.ppy], 
                          [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.array(intrinsics.coeffs, dtype=np.float32)

print("Camera Matrix:")
print(camera_matrix)
print("\nDistortion Coefficients:")
print(dist_coeffs)

# 停止 pipeline
pipeline.stop()