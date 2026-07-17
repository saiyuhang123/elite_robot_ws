import numpy as np
import cv2
from scipy.spatial.transform import Rotation as Rot
import json
import os

class EyeInHandCalibrator:
    def __init__(self):
        # 用于存储多组标定数据的列表
        self.R_gripper2base_list = []  # 机械臂末端相对于基座的旋转矩阵
        self.t_gripper2base_list = []  # 机械臂末端相对于基座的平移向量
        self.R_target2cam_list = []    # 标定板相对于相机的旋转矩阵
        self.t_target2cam_list = []    # 标定板相对于相机的平移向量
        
        # 最终标定出的相机到机械臂末端（Tool）的变换矩阵
        self.R_cam2tool = None
        self.t_cam2tool = None

    def add_sample(self, robot_pose, cam_rvec, cam_tvec):
        """
        向标定器添加一组配对样本。
        
        参数:
        robot_pose: 列表或数组 [x, y, z, rx, ry, rz] 
                    - x, y, z: 机械臂末端位置，单位：米 (m) (艾力特 SDK 读取的值)
                    - rx, ry, rz: 机械臂末端欧拉角，单位：弧度 (rad) (艾力特 SDK 读取的值)
        cam_rvec:   OpenCV 检测到的标定板旋转向量 (Rotation Vector)，通常由 cv2.solvePnP 或 ArUco 检测输出
        cam_tvec:   OpenCV 检测到的标定板平移向量，单位：米 (m) 
                    - 警告：如果 OpenCV 输出的单位是毫米 (mm)，必须除以 1000.0 转换为米！
        """
        # 1. 解析机械臂末端位姿 (Tool -> Base)
        x, y, z, rx, ry, rz = robot_pose
        t_gripper2base = np.array([x, y, z]).reshape(3, 1)
        
        # 将艾力特的欧拉角 (外在XYZ / 固有ZYX) 转换为 3x3 旋转矩阵
        r_gripper = Rot.from_euler('xyz', [rx, ry, rz], degrees=False)
        R_gripper2base = r_gripper.as_matrix()
        
        # 2. 解析相机观测到的标定板位姿 (Target -> Cam)
        # 用罗德里格斯变换将 3x1 旋转向量转为 3x3 旋转矩阵
        R_target2cam, _ = cv2.Rodrigues(np.array(cam_rvec))
        t_target2cam = np.array(cam_tvec).reshape(3, 1)
        
        # 3. 存入列表
        self.R_gripper2base_list.append(R_gripper2base)
        self.t_gripper2base_list.append(t_gripper2base)
        self.R_target2cam_list.append(R_target2cam)
        self.t_target2cam_list.append(t_target2cam)
        
        print(f"成功添加第 {len(self.R_gripper2base_list)} 组标定样本。")

    def solve(self, method=cv2.CALIB_HAND_EYE_TSAI):
        """
        计算手眼标定结果（求出相机相对于机械臂末端的位姿）。
        """
        if len(self.R_gripper2base_list) < 10:
            print(f"警告：当前样本数只有 {len(self.R_gripper2base_list)} 组。建议至少收集 15 组以上的数据以保证精度。")
            
        # 调用 OpenCV 核心手眼标定算法
        self.R_cam2tool, self.t_cam2tool = cv2.calibrateHandEye(
            self.R_gripper2base_list,
            self.t_gripper2base_list,
            self.R_target2cam_list,
            self.t_target2cam_list,
            method=method
        )
        
        print("\n================ 标定计算完成 ================")
        print("相机相对于末端的平移向量 t_cam2tool (单位: 米):\n", self.t_cam2tool)
        print("相机相对于末端的旋转矩阵 R_cam2tool:\n", self.R_cam2tool)
        print("=============================================\n")
        return self.R_cam2tool, self.t_cam2tool

    def save_result(self, filepath="hand_eye_result.json"):
        """
        将标定结果保存为 JSON 文件，便于上位机抓取程序直接加载。
        """
        if self.R_cam2tool is None or self.t_cam2tool is None:
            print("错误：尚未进行标定，无法保存结果。")
            return
            
        result = {
            "R_cam2tool": self.R_cam2tool.tolist(),
            "t_cam2tool": self.t_cam2tool.tolist()
        }
        with open(filepath, 'w') as f:
            json.dump(result, f, indent=4)
        print(f"标定结果已成功保存至: {filepath}")

    def load_result(self, filepath="hand_eye_result.json"):
        """
        加载之前保存的标定结果。
        """
        if not os.path.exists(filepath):
            print(f"错误：找不到标定文件 {filepath}")
            return False
            
        with open(filepath, 'r') as f:
            result = json.load(f)
        self.R_cam2tool = np.array(result["R_cam2tool"])
        self.t_cam2tool = np.array(result["t_cam2tool"])
        print(f"成功加载标定文件: {filepath}")
        return True

    def transform_point_to_base(self, current_robot_pose, p_cam):
        """
        运行时转换：将 YOLO+深度相机 检测到的物体三维坐标，转换为机械臂基座下的三维坐标。
        
        参数:
        current_robot_pose: 列表 [x, y, z, rx, ry, rz] 
                            - 抓取瞬间，艾力特机械臂当前的末端位姿 (米, 弧度)
        p_cam:              列表或数组 [X_c, Y_c, Z_c] 
                            - 物体在相机坐标系下的三维物理坐标 (单位：米)
        
        返回:
        p_base:             np.array [X_b, Y_b, Z_b] 
                            - 物体在机械臂基座坐标系下的坐标 (单位：米)
        """
        if self.R_cam2tool is None or self.t_cam2tool is None:
            raise ValueError("错误：未检测到有效的标定结果，请先标定或加载标定文件！")

        # 1. 提取当前机械臂末端坐标系的 R 和 t
        x, y, z, rx, ry, rz = current_robot_pose
        t_tool2base = np.array([x, y, z]).reshape(3, 1)
        r_tool = Rot.from_euler('xyz', [rx, ry, rz], degrees=False)
        R_tool2base = r_tool.as_matrix()

        # 2. 将点转换为 3x1 列向量
        P_c = np.array(p_cam).reshape(3, 1)

        # 3. 步骤一：相机坐标系 -> 机械臂末端坐标系
        P_tool = np.dot(self.R_cam2tool, P_c) + self.t_cam2tool

        # 4. 步骤二：机械臂末端坐标系 -> 机械臂基座坐标系
        P_base = np.dot(R_tool2base, P_tool) + t_tool2base

        return P_base.flatten()


# ==========================================
# 演示与自检测试代码（您可以直接运行此文件进行测试）
# ==========================================
if __name__ == "__main__":
    calibrator = EyeInHandCalibrator()
    
    # 模拟生成 15 组标定数据（真实情况下由您的程序在不同姿态下录入）
    print("--- 正在模拟录入 15 组样本数据 ---")
    for i in range(15):
        # 模拟机械臂位姿 (单位：米，弧度)
        # 假设机械臂在 X=0.3m, Y=0.1m, Z=0.4m 附近晃动
        mock_robot_pose = [
            0.3 + 0.05 * np.sin(i), 
            0.1 + 0.05 * np.cos(i), 
            0.4 + 0.02 * np.sin(i * 2),
            0.1 * np.sin(i), 
            0.1 * np.cos(i), 
            -0.785 + 0.1 * np.sin(i) # 绕Z轴大概转-45度
        ]
        
        # 模拟相机检测到的标定板位姿 (单位：米)
        mock_cam_rvec = [0.01 * np.sin(i), 0.02 * np.cos(i), 1.57 + 0.01 * i]
        mock_cam_tvec = [0.05 * np.sin(i), -0.05 * np.cos(i), 0.5 + 0.01 * i] # 标定板在相机前方约0.5米
        
        calibrator.add_sample(mock_robot_pose, mock_cam_rvec, mock_cam_tvec)

    # 计算标定结果
    calibrator.solve()
    
    # 保存结果
    json_path = "test_hand_eye_result.json"
    calibrator.save_result(json_path)
    
    # ------------------ 运行时模拟 ------------------
    print("\n--- 模拟运行时 YOLO 抓取转换 ---")
    # 实例化一个新的转换器，加载刚才保存的标定结果
    runtime_processor = EyeInHandCalibrator()
    if runtime_processor.load_result(json_path):
        
        # 1. 假设我们在机械臂当前位姿读取到：
        current_pose = [0.35, 0.12, 0.38, 0.05, 0.08, -0.72] # 机械臂当前位姿 (米, 弧度)
        
        # 2. YOLO 和 深度相机 检测到物体在相机坐标系下的坐标为：
        # 比如：物体在相机正前方 0.45米，偏右 0.02米，偏下 0.05米
        object_in_cam = [0.02, -0.05, 0.45] # (单位: 米)
        
        # 3. 进行转换
        object_in_base = runtime_processor.transform_point_to_base(current_pose, object_in_cam)
        
        print("\n[YOLO 检测输入] 物体在相机坐标系下:", object_in_cam)
        print("[转换计算输出] 物体在机械臂底座坐标系下 (发送给SDK运动的值):")
        print(f"X = {object_in_base[0]:.4f} m ({object_in_base[0]*1000:.1f} mm)")
        print(f"Y = {object_in_base[1]:.4f} m ({object_in_base[1]*1000:.1f} mm)")
        print(f"Z = {object_in_base[2]:.4f} m ({object_in_base[2]*1000:.1f} mm)")

    # 清理测试生成的临时文件
    if os.path.exists(json_path):
        os.remove(json_path)