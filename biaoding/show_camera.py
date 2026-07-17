import pyrealsense2 as rs
import numpy as np
import cv2

# 1. 创建工作流并配置相机分辨率
pipeline = rs.pipeline()
config = rs.config()

# D435 推荐分辨率为 640x480 或 848x480
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# 2. 启动相机
pipeline.start(config)

# 3. 创建深度彩色化工具（将16位深度值映射成肉眼易看的彩虹色/伪彩色）
colorizer = rs.colorizer()

try:
    print("程序已启动。在图像窗口上按 'q' 键可退出。")
    while True:
        # 等待一组帧数据
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # 将深度图转为彩虹色
        colorized_depth = colorizer.colorize(depth_frame)

        # 将数据转换为 numpy 数组以供 OpenCV 显示
        depth_image = np.asanyarray(colorized_depth.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # 将彩色图和深度图横向拼接在一起显示
        images = np.hstack((color_image, depth_image))

        # 显示窗口
        cv2.namedWindow('RealSense D435 - Live', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense D435 - Live', images)

        # 按 'q' 键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # 停止相机工作流并关闭窗口
    pipeline.stop()
    cv2.destroyAllWindows()