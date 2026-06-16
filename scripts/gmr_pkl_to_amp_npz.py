import argparse
import pickle
from pathlib import Path

import numpy as np


def normalize_quat_xyzw(quat):
    norm = np.linalg.norm(quat, axis=-1, keepdims=True)
    return quat / np.clip(norm, 1e-8, None)


def quat_conjugate_xyzw(quat):
    out = quat.copy()
    out[..., :3] *= -1.0
    return out


def quat_mul_xyzw(a, b):
    ax, ay, az, aw = np.moveaxis(a, -1, 0)
    bx, by, bz, bw = np.moveaxis(b, -1, 0)
    return np.stack(
        (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ),
        axis=-1,
    )


def finite_difference(values, fps):
    vel = np.zeros_like(values, dtype=np.float32)
    if len(values) < 2:
        return vel

    vel[:-1] = (values[1:] - values[:-1]) * fps
    vel[-1] = vel[-2]
    return vel


def angular_velocity_xyzw(rot, fps):
    ang_vel = np.zeros((rot.shape[0], 3), dtype=np.float32)
    if len(rot) < 2:
        return ang_vel

    delta = quat_mul_xyzw(rot[1:], quat_conjugate_xyzw(rot[:-1]))
    delta = normalize_quat_xyzw(delta)

    # Use the shortest quaternion arc before converting to axis-angle velocity.
    flip = delta[:, 3] < 0.0
    delta[flip] *= -1.0

    xyz = delta[:, :3]
    w = np.clip(delta[:, 3], -1.0, 1.0)
    sin_half = np.linalg.norm(xyz, axis=-1)
    angle = 2.0 * np.arctan2(sin_half, w)

    axis = np.zeros_like(xyz)
    valid = sin_half > 1e-8
    axis[valid] = xyz[valid] / sin_half[valid, None]
    ang_vel[:-1] = axis * angle[:, None] * fps
    ang_vel[-1] = ang_vel[-2]
    return ang_vel.astype(np.float32)


def convert_pkl_to_amp_npz(input_path, output_path):
    with open(input_path, "rb") as f:
        motion_data = pickle.load(f)

    fps = int(motion_data["fps"])
    root_pos = np.asarray(motion_data["root_pos"], dtype=np.float32)
    root_rot = normalize_quat_xyzw(np.asarray(motion_data["root_rot"], dtype=np.float32))
    dof_pos = np.asarray(motion_data["dof_pos"], dtype=np.float32)

    if root_pos.shape[0] != root_rot.shape[0] or root_pos.shape[0] != dof_pos.shape[0]:
        raise ValueError("root_pos, root_rot, and dof_pos must have the same frame count")

    root_vel = finite_difference(root_pos, fps)
    root_ang_vel = angular_velocity_xyzw(root_rot, fps)
    dof_vel = finite_difference(dof_pos, fps)
    motion = np.concatenate((root_pos, root_rot, dof_pos), axis=-1).astype(np.float32)

    np.savez(
        output_path,
        fps=np.array(fps, dtype=np.int32),
        dt=np.array(1.0 / fps, dtype=np.float32),
        quat_order=np.array("xyzw"),
        root_pos=root_pos,
        root_rot=root_rot,
        root_vel=root_vel,
        root_ang_vel=root_ang_vel,
        dof_pos=dof_pos,
        dof_vel=dof_vel,
        motion=motion,
        source_path=np.array(str(input_path)),
    )


def main():
    parser = argparse.ArgumentParser(description="Convert a GMR robot motion pickle to an AMP-style npz.")
    parser.add_argument("--input", required=True, type=Path, help="Input GMR .pkl file")
    parser.add_argument("--output", type=Path, help="Output .npz path")
    args = parser.parse_args()

    output_path = args.output or args.input.with_suffix(".npz")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    convert_pkl_to_amp_npz(args.input, output_path)
    print(f"Saved AMP-style npz to {output_path}")


if __name__ == "__main__":
    main()
