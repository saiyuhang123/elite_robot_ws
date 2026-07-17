import numpy as np
import cv2
from scipy.spatial.transform import Rotation as Rot
import json

# ==========================================================
# 1. 填入您重新采集的数据（以下仍以您的 10 组数据做演示）
# ==========================================================
raw_samples = [
    # 第一组
    {
        "robot_pos": [0.24173931516572134, 0.21449088642101383, 0.9066699927614154],
        "robot_ori": [-0.7228841821983563, 0.21234446276112098, -0.6224315362832454, 0.21196054090840502], # [x, y, z, w]
        "cam_rvec": [0.97323386, 2.63988158, 0.85045972],
        "cam_tvec": [-0.13951264, -0.02543239, 0.49345122]
    },
    # 第二组
    {
        "robot_pos": [0.20241110066821844, 0.22909371783722604, 0.6927258106410579],
        "robot_ori": [-0.611724860553144, 0.03400734530846559, -0.5806921868246063, 0.5361275777342215],
        "cam_rvec": [0.24377532, 3.01422962, 0.19456078],
        "cam_tvec": [0.02678417, -0.0505135, 0.59499536]
    },
    # 第三组
    {
        "robot_pos": [0.023670305644384302, 0.17775953313764603, 0.7963570618515601],
        "robot_ori": [-0.6978935677358387, 0.034351222127582424, -0.4647889029563903, 0.5438159958477492],
        "cam_rvec": [-0.2656203, -2.81391359, -0.50841448],
        "cam_tvec": [0.2378655, -0.09767548, 0.71831324]
    },
    # 第四组
    {
        "robot_pos": [0.19396041106779827, 0.282915488563569, 0.722600670907207],
        "robot_ori": [-0.637467602243677, 0.10619156630593025, -0.6546725620651682, 0.392125290964787],
        "cam_rvec": [0.45577479, 2.75914518, 0.26712346],
        "cam_tvec": [-0.11766985, -0.08451495, 0.55451504]
    },
    # 第五组
    {
        "robot_pos": [0.13560225490822822, 0.25168991054780265, 0.9257055243475667],
        "robot_ori": [-0.7170823035653688, 0.20385112375523, -0.5878973883715257, 0.3140292183906677],
        "cam_rvec": [0.90970611, 2.81103794, 0.79931428],
        "cam_tvec": [-0.017078, -0.06568766, 0.623108]
    },
    # 第六组
    {
        "robot_pos": [0.4491830948862183, 0.27820884618941016, 0.7980179881141771],
        "robot_ori": [-0.6435226421673744, 0.21402405378369485, -0.589568725819971, 0.43872660160392773],
        "cam_rvec": [0.90332253, 2.91186113, 0.40742623],
        "cam_tvec": [-0.0424050263, -0.000309002361, 0.313219168]
    },
    # 第七组
    {
        "robot_pos": [0.5155586654272478, 0.033763715423332404, 0.8048036744221689],
        "robot_ori": [-0.6731260699866999, 0.019335211492856982, -0.3399710477021208, 0.6564656352202369],
        "cam_rvec": [-0.2883664, -2.44199087, -0.28692043],
        "cam_tvec": [0.01701158, 0.01455618, 0.38130399]
    },
    # 第八组
    {
        "robot_pos": [-0.059359661475029996, 0.16502974517339927, 0.9876408646550845],
        "robot_ori": [-0.6374206507151009, 0.21442809601010096, -0.5755147223933629, 0.46529378890321604],
        "cam_rvec": [0.89598167, 2.92321809, 0.42918089],
        "cam_tvec": [0.04687332, 0.18817291, 0.85190992]
    },
    # 第九组
    {
        "robot_pos": [-0.09111064260347279, 0.3346492575828119, 0.6596529548615048],
        "robot_ori": [-0.5316151936318414, -0.16107104002845465, -0.724108577677305, 0.40878866630336286],
        "cam_rvec": [-0.59752335, 2.75974305, 0.25034228],
        "cam_tvec": [-0.06831812, -0.0116016, 0.88917462]
    },
    # 第十组
    {
        "robot_pos": [0.1732637640498682, 0.6074771810007826, 0.5532635984329438],
        "robot_ori": [-0.40660803331764866, -0.18027372038471912, -0.8830136605715023, 0.14986049587640438],
        "cam_rvec": [-0.89162094, 2.11307087, 0.2748965],
        "cam_tvec": [-0.09448966, -0.12233989, 0.6984504]
    }
]

# ==========================================================
# 2. 解析与矩阵转换
# ==========================================================
R_g2b_list, t_g2b_list = [], []
R_t2c_list, t_t2c_list = [], []

for sample in raw_samples:
    t_g2b = np.array(sample["robot_pos"]).reshape(3, 1)
    # 四元数 [x, y, z, w] -> 3x3 矩阵
    R_g2b = Rot.from_quat(sample["robot_ori"]).as_matrix()
    
    t_t2c = np.array(sample["cam_tvec"]).reshape(3, 1)
    # 旋转向量 -> 3x3 矩阵
    R_t2c, _ = cv2.Rodrigues(np.array(sample["cam_rvec"]))
    
    R_g2b_list.append(R_g2b)
    t_g2b_list.append(t_g2b)
    R_t2c_list.append(R_t2c)
    t_t2c_list.append(t_t2c)

# ==========================================================
# 3. 手眼标定计算
# ==========================================================
R_c2g, t_c2g = cv2.calibrateHandEye(
    R_g2b_list, t_g2b_list, R_t2c_list, t_t2c_list,
    method=cv2.CALIB_HAND_EYE_TSAI
)

# ==========================================================
# 4. 核心：残差自检计算 (Residual Check)
# ==========================================================
# 原理：根据 AX=XB，在标定准确的前提下，将所有位姿转换到同一个基座坐标系下，
# 标定板在基座下的估计位置 H_t2b 应该是完全重合的。
T_t2b_list = []
estimated_positions = []
estimated_rotations = []

for i in range(len(R_g2b_list)):
    # 构造 H_g2b
    H_g2b = np.eye(4)
    H_g2b[:3, :3] = R_g2b_list[i]
    H_g2b[:3, 3] = t_g2b_list[i].flatten()
    
    # 构造 H_c2g (即计算出的手眼标定矩阵)
    H_c2g = np.eye(4)
    H_c2g[:3, :3] = R_c2g
    H_c2g[:3, 3] = t_c2g.flatten()
    
    # 构造 H_t2c
    H_t2c = np.eye(4)
    H_t2c[:3, :3] = R_t2c_list[i]
    H_t2c[:3, 3] = t_t2c_list[i].flatten()
    
    # 标定板在基座坐标系下的位置估算：H_t2b = H_g2b * H_c2g * H_t2c
    H_t2b = np.dot(H_g2b, np.dot(H_c2g, H_t2c))
    T_t2b_list.append(H_t2b)
    
    estimated_positions.append(H_t2b[:3, 3])
    estimated_rotations.append(Rot.from_matrix(H_t2b[:3, :3]))

# A. 计算平移偏差 (到平均位置的欧氏距离)
estimated_positions = np.array(estimated_positions)
mean_position = np.mean(estimated_positions, axis=0)
t_errors_m = np.linalg.norm(estimated_positions - mean_position, axis=1)
mean_t_error_mm = np.mean(t_errors_m) * 1000.0
max_t_error_mm = np.max(t_errors_m) * 1000.0

# B. 计算旋转偏差 (各姿态与平均姿态的相对偏角)
# 注意：scipy 1.15.x 的 Rot.mean() 有 bug 会导致段错误，使用手动实现
def mean_rotation_manual(rotations):
    """手动计算平均旋转（四元数均值法）"""
    quats = np.array([r.as_quat() for r in rotations])  # [x, y, z, w] 格式
    # 确保所有四元数在同一半球（处理双覆盖性）
    for i in range(1, len(quats)):
        if np.dot(quats[0], quats[i]) < 0:
            quats[i] = -quats[i]
    # 计算平均四元数
    mean_quat = np.mean(quats, axis=0)
    # 归一化
    mean_quat = mean_quat / np.linalg.norm(mean_quat)
    return Rot.from_quat(mean_quat)

mean_rotation = mean_rotation_manual(estimated_rotations)

r_errors_deg = []
for r in estimated_rotations:
    # 计算两个姿态间的相对偏角（轴角大小）
    relative_rot = r * mean_rotation.inv()
    angle_rad = np.linalg.norm(relative_rot.as_rotvec())
    r_errors_deg.append(np.degrees(angle_rad))

mean_r_error_deg = np.mean(r_errors_deg)
max_r_error_deg = np.max(r_errors_deg)

# ==========================================================
# 5. 输出评估报告
# ==========================================================
print("\n================== 标定残差自检报告 ==================")
print(f"平均平移残差: {mean_t_error_mm:.2f} 毫米 (最大: {max_t_error_mm:.2f} 毫米)")
print(f"平均旋转残差: {mean_r_error_deg:.2f} 度   (最大: {max_r_error_deg:.2f} 度)")
print("-----------------------------------------------------")

# 评估标准：常规毫米级高精度抓取建议平移残差 < 5mm，旋转残差 < 1.0°
if mean_t_error_mm < 5.0 and mean_r_error_deg < 1.0:
    print("【 评估结论 】：★ 优秀 ★")
    print("数据自洽度极高，此标定结果非常精准，可直接用于实际定位与抓取。")
elif mean_t_error_mm < 15.0 and mean_r_error_deg < 2.5:
    print("【 评估结论 】：▲ 一般 ▲")
    print("精度勉强可用，适合对定位精度要求不高的粗放型抓取场景。")
else:
    print("【 评估结论 】：❌ 严重不合格 ❌")
    print("残差过大！数据在物理/几何上不自洽。此矩阵不可用，请废弃并重新采集！")
print("=====================================================\n")