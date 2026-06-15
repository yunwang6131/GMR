# 主要支持数据源：
SMPL-X: AMASS / OMOMO
BVH: LAFAN1 / Nokov / Xsens
GVHMR: 单目视频提取出来的人体姿态
FBX: OptiTrack 导出的动作
PICO / XRobot: 实时遥操作数据

输入：原始人体动作数据
输出：能被脚本读取的人体动作文件
比如：
AMASS: .npz
LAFAN1: .bvh
GVHMR: hmr4d_results.pt
FBX: .fbx 先转中间 .pkl
Xsens: .bvh 或 UDP 实时流

[motion_retarget.py (line 8)]
核心重定向类 GeneralMotionRetargeting

[robot_motion_viewer.py (line 34)]
MuJoCo 播放器 RobotMotionViewer

[params.py (line 1)]
机器人、XML、IK config 的注册表

[kinematics_model.py (line 57)]
正运动学，用于从 dof_pos 算 link 位置

[data_loader.py (line 4)]
读取保存后的 robot motion .pkl

utils/
各种人体数据格式读取器，比如 SMPL-X、LAFAN1、Xsens
##  单个 BVH 人体动作
# 走路
python3 scripts/bvh_to_robot.py \
  --bvh_file motion_data/lafan1/bvh/walk1_subject1.bvh \
  --format lafan1 \
  --robot d2_smplify \
  --save_path outputs/walk1_subject1.pkl \
  --rate_limit

python3 scripts/bvh_to_robot.py \
  --bvh_file motion_data/lafan1/bvh/fallAndGetUp1_subject1.bvh \
  --format lafan1 \
  --robot d2_smplify \
  --save_path outputs/fallAndGetUp1_subject1.pkl \
  --rate_limit

# 一个文件夹里的所有 .bvh
python3 scripts/bvh_to_robot_dataset.py \
  --src_folder motion_data/lafan1/bvh \
  --tgt_folder motion_data/LAFAN1_d2_gmr \
  --robot d2_smplify

# 可视化
python3 scripts/vis_robot_motion.py \
  --robot d2_smplify \
  --robot_motion_path outputs/d2_motion.pkl

# 先把改动临时存起来，再拉取

git stash push -u -m "save local GMR changes before pulling motion data"

然后拉取远程：

git pull

# 恢复改动

git stash pop

# 调参
general_motion_retargeting/ik_configs/bvh_lafan1_to_d2_smplify.json


"human_scale_table"
控制人体各段缩放比例。比如腿太长、手太远、身体比例不对，调这里。
"ik_match_table1"
"ik_match_table2"
这是核心。每一项类似：
"left_ankle_roll_link": [
    "LeftFootMod",
    50,
    10,
    [0.0, 0.0, 0.0],
    [-0.70710678, 0.70710678, 0.0, 0.0]
]
含义是：
机器人 link: left_ankle_roll_link
对应人体部位: LeftFootMod
位置权重: 50
旋转权重: 10
位置 offset: [0,0,0]
旋转 offset: 四元数 wxyz
你之后主要手动调：
位置权重
旋转权重
位置 offset
旋转 offset
human_scale_table
ground_height
经验上：
脚滑、脚位置不准：提高脚的 position weight
手臂姿态怪：调肩/肘/腕的 rotation offset
身体弯得怪：调 truck_link / torso 相关权重
整体比例怪：调 human_scale_table
机器人浮空/插地：调 ground_height 或后处理 root 高度
pkl 是什么格式
它是 Python pickle，不是通用文本格式。里面是一个 dict，通常长这样：
{
    "fps": 30,
    "root_pos": np.ndarray,      # [T, 3]
    "root_rot": np.ndarray,      # [T, 4], xyzw 保存
    "dof_pos": np.ndarray,       # [T, 29] 对 D2 来说
    "local_body_pos": ...,
    "link_body_list": ...
}
对于 D2，加了 freejoint 后完整 qpos 是：
root_pos 3维
root_rot 4维
dof_pos 29维
也就是每帧总共：
3 + 4 + 29 = 36

| 文件前缀            | 含义        |
| --------------- | --------- |
| walk            | 走路        |
| run             | 跑步        |
| sprint          | 冲刺        |
| jump            | 跳跃        |
| dance           | 跳舞        |
| aiming          | 瞄准、持枪瞄准   |
| fight           | 格斗        |
| fightAndSports  | 格斗和运动动作混合 |
| ground          | 地面动作      |
| obstacles       | 跨越障碍      |
| push            | 被推        |
| pushAndStumble  | 被推后踉跄     |
| pushAndFall     | 被推后摔倒     |
| fallAndGetUp    | 摔倒并起身     |
| multipleActions | 多种动作组合    |
