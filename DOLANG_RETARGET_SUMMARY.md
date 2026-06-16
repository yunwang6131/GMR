# Dolang D2 / GMR 重定向总结

本文记录这次把 GMR 接到 Dolang D2 / `d2_smplify` 时遇到的问题、解决方法、运行流程，以及后续调参和导出 BeyondMimic 数据时需要注意的地方。

## 1. 项目目标

目标是把人体动作数据重定向到 Dolang D2 机器人模型：

```text
LAFAN1 BVH 人体动作
    -> GMR 解析成人体关节目标
    -> MuJoCo + mink IK 求解 D2 机器人 qpos
    -> 输出 D2 的 root_pos / root_rot / dof_pos
    -> 保存 pkl
    -> 可转 CSV 给 BeyondMimic 使用
```

当前 D2 在 GMR 里的机器人名是：

```text
d2_smplify
```

主要模型文件：

```text
assets/d2_smplify/D2_QIAOJIE_smplify.xml
```

LAFAN1 到 D2 的 IK 配置文件：

```text
general_motion_retargeting/ik_configs/bvh_lafan1_to_d2_smplify.json
```

## 2. 已接通的数据路径

### LAFAN1 -> D2

这是当前主要使用路径：

```bash
python3 scripts/bvh_to_robot.py \
  --bvh_file motion_data/lafan1/bvh/walk1_subject1.bvh \
  --format lafan1 \
  --robot d2_smplify \
  --save_path outputs/walk1_subject1.pkl \
  --rate_limit
```

含义：

```text
--bvh_file     输入 LAFAN1 原始 BVH 动作
--format       使用 lafan1 解析器
--robot        目标机器人 D2
--save_path    保存 D2 重定向结果
--rate_limit   按动作 FPS 播放，而不是尽快播放
```

输出：

```text
outputs/walk1_subject1.pkl
```

这个 pkl 是 D2 机器人动作，不是原始人体动作。

### Xsens BVH -> D2

之前也补过 Xsens 到 D2 的初版配置：

```text
general_motion_retargeting/ik_configs/bvh_xsens_to_d2_smplify.json
```

对应脚本：

```bash
python3 scripts/xsens_bvh_to_robot.py \
  --robot d2_smplify \
  --scale 0.01 \
  --reset_to_zero \
  --bvh_format 3DSM \
  --bvh_file assets/xsens_bvh_test/251021_04_boxing_120Hz_cm_3DsMax.bvh \
  --save_path outputs/d2_xsens_motion.pkl
```

注意：Xsens BVH 不能用 `scripts/bvh_to_robot.py --format lafan1` 解析。

## 3. 遇到的问题和解决方法

### 问题 1：D2 MuJoCo 模型没有 root freejoint

现象：

GMR 默认机器人是 floating base，代码里按下面格式写 qpos：

```text
qpos[:3]   = root_pos
qpos[3:7]  = root_rot
qpos[7:]   = dof_pos
```

如果 D2 的 MJCF 根节点没有 `freejoint`，那么 qpos 只有 29 个关节角，没有前 7 维 root 位姿，会导致维度或语义错位。

解决：

在 D2 XML 的根 body 下加：

```xml
<body name="base_link">
  <freejoint name="root"/>
```

修改位置：

```text
assets/d2_smplify/D2_QIAOJIE_smplify.xml
```

期望结果：

```text
D2 qpos = 7 + 29 = 36
root_pos 3
root_rot 4
dof_pos 29
```

### 问题 2：把 Xsens BVH 当成 LAFAN1 BVH 解析

错误命令：

```bash
python3 scripts/bvh_to_robot.py \
  --bvh_file assets/xsens_bvh_test/251021_04_boxing_120Hz_cm_3DsMax.bvh \
  --format lafan1 \
  --robot d2_smplify
```

报错：

```text
KeyError: 'LeftFoot'
```

原因：

`assets/xsens_bvh_test/...` 是 Xsens BVH，不是 LAFAN1 BVH。LAFAN1 解析器会找 `LeftFoot`、`LeftToe` 等 LAFAN1 骨骼名，Xsens 的骨骼命名不同。

解决：

LAFAN1 数据走：

```bash
python3 scripts/bvh_to_robot.py --format lafan1 ...
```

Xsens 数据走：

```bash
python3 scripts/xsens_bvh_to_robot.py ...
```

### 问题 3：以为一个 BVH 会生成很多 pkl

实际关系：

```text
一个 .bvh 输入 -> 一个 .pkl 输出
```

例如：

```text
motion_data/lafan1/bvh/dance1_subject1.bvh
    -> outputs/dance1_subject1.pkl
```

如果一个 BVH 里有 3945 帧，输出 pkl 也是一条包含约 3945 帧的长动作，不是 3945 个文件。

### 问题 4：Blender 里只看到 250 帧

现象：

MuJoCo 播放很长，但 Blender 打开 BVH 只看到 250 帧。

原因：

Blender 默认 Timeline End 通常是 250，而 LAFAN1 的某些动作本身远超 250 帧，例如：

```text
dance1_subject1.bvh
Frames: 3945
Frame Time: 0.033333
```

解决：

在 Blender 底部时间轴把：

```text
End: 250
```

改成：

```text
End: 3945
```

### 问题 5：MuJoCo 里动作抖动

可能原因：

```text
1. IK 权重太强，尤其 torso / shoulder / wrist rotation weight
2. D2 机器人结构和人体骨架差异大
3. 没有启用速度限制或平滑
4. 人体参考目标本身有抖动
```

判断方法：

运行 `bvh_to_robot.py` 时，MuJoCo 里会显示红绿蓝坐标轴。这些是人体参考目标。

```text
红绿蓝坐标轴也抖 -> 源动作或人体解析问题
坐标轴稳定但 D2 抖 -> IK 配置、权重、offset 或关节限制问题
```

优先调：

```text
general_motion_retargeting/ik_configs/bvh_lafan1_to_d2_smplify.json
```

常见调法：

```text
脚抖：降低脚 position / rotation weight
手臂抖：降低 shoulder / elbow / wrist rotation weight
躯干抖：降低 truck_link rotation weight
比例不对：调 human_scale_table
浮空或插地：调 ground_height 或后处理 root 高度
```

### 问题 6：手腕总是 90 度往外撇

现象：

`*_wrist_pitch_joint` 总是约 90 度往外撇，原始动画里手臂应该自然下垂。

原因分析：

D2 手腕链路是：

```text
wrist_pitch_link
    -> wrist_roll_link
        -> wrist_camera_link
```

`wrist_roll_link` 相对 `wrist_pitch_link` 有明显侧向和向下偏移。如果让 `wrist_roll_link` 去追人体 `LeftHand/RightHand`，IK 可能会让 `wrist_pitch_joint` 大幅外撇来补这个 frame 偏移。

解决思路：

在 LAFAN1 配置中，把手部目标从：

```json
"left_wrist_roll_link": ["LeftHand", ...]
"right_wrist_roll_link": ["RightHand", ...]
```

改成：

```json
"left_wrist_pitch_link": ["LeftHand", ...]
"right_wrist_pitch_link": ["RightHand", ...]
```

并且先不强追手腕旋转：

```json
"left_wrist_pitch_link": ["LeftHand", 10, 0, ...]
"right_wrist_pitch_link": ["RightHand", 10, 0, ...]
```

注意：

`ik_match_table1` 中不要把某个 body 的位置权重和旋转权重都设成 0，否则 GMR 可能不会为这个 body 建 offset 表，导致：

```text
KeyError: 'LeftHand'
```

因此 table1 里可以保留很小的位置权重：

```json
"left_wrist_pitch_link": ["LeftHand", 1, 0, ...]
"right_wrist_pitch_link": ["RightHand", 1, 0, ...]
```

### 问题 7：KeyError: 'LeftHand'

现象：

```text
KeyError: 'LeftHand'
```

原因：

在 `ik_match_table1` 中把 `LeftHand/RightHand` 对应 task 的位置和旋转权重都设为 0，GMR 没有创建 `pos_offsets1/rot_offsets1`，但后续 `offset_human_data` 又会处理这个 body。

解决：

不要让 table1 中该 body 的两个权重都为 0。可以设置：

```json
["LeftHand", 1, 0, ...]
["RightHand", 1, 0, ...]
```

### 问题 8：GLXBadDrawable

现象：

```text
X Error of failed request: GLXBadDrawable
```

原因：

通常是 Python 异常后 MuJoCo viewer 被非正常关闭带出的显示错误，不是根因。

解决：

先看前面的 Python traceback。比如这次真正根因是：

```text
KeyError: 'LeftHand'
```

## 4. 单个动作重定向流程

推荐先用简单动作测试，例如 walk：

```bash
python3 scripts/bvh_to_robot.py \
  --bvh_file motion_data/lafan1/bvh/walk1_subject1.bvh \
  --format lafan1 \
  --robot d2_smplify \
  --save_path outputs/walk1_subject1.pkl \
  --rate_limit
```

也可以测试 push / fall 类动作：

```bash
python3 scripts/bvh_to_robot.py \
  --bvh_file motion_data/lafan1/bvh/pushAndStumble1_subject2.bvh \
  --format lafan1 \
  --robot d2_smplify \
  --save_path outputs/pushAndStumble1_subject2.pkl \
  --rate_limit
```

输出 pkl：

```text
outputs/pushAndStumble1_subject2.pkl
```

## 5. 批量重定向流程

如果单个动作效果可以，再批量处理整个 LAFAN1 文件夹：

```bash
python3 scripts/bvh_to_robot_dataset.py \
  --src_folder motion_data/lafan1/bvh \
  --tgt_folder motion_data/LAFAN1_d2_gmr \
  --robot d2_smplify
```

输出关系：

```text
motion_data/lafan1/bvh/walk1_subject1.bvh
    -> motion_data/LAFAN1_d2_gmr/walk1_subject1.pkl

motion_data/lafan1/bvh/dance1_subject1.bvh
    -> motion_data/LAFAN1_d2_gmr/dance1_subject1.pkl
```

## 6. pkl 格式

GMR 输出的 pkl 是 Python pickle，里面是一个 dict：

```python
{
    "fps": 30,
    "root_pos": np.ndarray,      # [T, 3]
    "root_rot": np.ndarray,      # [T, 4], 通常按 xyzw 保存
    "dof_pos": np.ndarray,       # [T, 29]，D2 的 29 个关节角
    "local_body_pos": ...,
    "link_body_list": ...
}
```

D2 每一帧的完整 motion 向量：

```text
root_pos 3
root_rot 4
dof_pos 29
总计 36 维
```

## 7. 转 CSV 给 BeyondMimic

GMR 自带转换脚本：

```bash
python3 scripts/batch_gmr_pkl_to_csv.py \
  --folder outputs
```

如果输入是：

```text
outputs/walk1_subject1.pkl
```

输出是：

```text
outputs/csv/walk1_subject1.csv
```

CSV 每一行是一帧：

```text
root_pos(3), root_rot(4), dof_pos(29)
```

D2 总共 36 列：

```text
x, y, z, qx, qy, qz, qw, joint_0, ..., joint_28
```

## 8. BeyondMimic 使用注意事项

GMR 的 CSV 没有表头，BeyondMimic 不是从 CSV 里自动知道机器人和关节顺序的。

BeyondMimic 会按自己的机器人环境解释：

```text
第 0-2 列：root position
第 3-6 列：root quaternion
第 7 列之后：按 BeyondMimic 环境的 dof 顺序排列的关节角
```

因此必须确认：

```text
GMR 导出的 D2 dof_pos 顺序
==
BeyondMimic 中 D2 的 joint / dof 顺序
```

如果不一致，需要重排 CSV 的 `dof_pos` 列。

还要确认 root quaternion 顺序：

```text
GMR CSV 通常是 xyzw
如果 BeyondMimic 需要 wxyz，需要转换
```

## 9. 调参文件说明

主要调：

```text
general_motion_retargeting/ik_configs/bvh_lafan1_to_d2_smplify.json
```

配置项格式：

```json
"robot_link": [
    "human_body",
    position_weight,
    rotation_weight,
    position_offset,
    rotation_offset
]
```

示例：

```json
"left_ankle_roll_link": [
    "LeftFootMod",
    50,
    10,
    [0.0, 0.0, 0.0],
    [-0.70710678, 0.70710678, 0.0, 0.0]
]
```

含义：

```text
robot_link       D2 要追踪的 link
human_body       人体参考目标
position_weight  位置权重
rotation_weight  旋转权重
position_offset  位置偏移
rotation_offset  旋转偏移，四元数 wxyz
```

常见调参方向：

```text
动作抖：降低 rotation_weight
脚追不上：提高脚 position_weight 或调脚 offset
手臂外张：降低肩膀/手腕 rotation_weight，检查 wrist target link
比例不对：调 human_scale_table
目标点偏：调 position_offset
方向错：调 rotation_offset
```

## 10. 可视化检查方法

运行单文件重定向时：

```bash
python3 scripts/bvh_to_robot.py \
  --bvh_file motion_data/lafan1/bvh/walk1_subject1.bvh \
  --format lafan1 \
  --robot d2_smplify \
  --save_path outputs/walk1_subject1.pkl \
  --rate_limit
```

MuJoCo 里会同时显示：

```text
D2 机器人
红绿蓝人体参考坐标轴
```

判断：

```text
参考坐标轴平滑，D2 抖 -> IK 配置问题
参考坐标轴也抖 -> 源动作或人体解析问题
D2 某个部位偏离参考坐标轴 -> 调对应 link 的 weight / offset
```

查看原始 BVH 可用 Blender：

```text
File -> Import -> Motion Capture (.bvh)
```

如果只看到 250 帧，修改 Blender Timeline 的 `End` 值到 BVH 实际帧数。

## 11. Git 操作注意

当前本地有 D2 相关改动时，拉远程前建议先 stash：

```bash
git stash push -u -m "save local D2 retarget changes"
git pull
git stash apply
git status
```

确认没问题后再：

```bash
git stash drop
```

`-u` 会把未跟踪文件也保存起来，例如新建的 D2 IK config 或记录文档。

