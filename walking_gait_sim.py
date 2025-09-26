import pybullet as p
import pybullet_data
import time
import numpy as np
import math

# Define your 8-joint wkF gait data (54 frames * 8 joints = 432 values)
wkF = [
    54, 0, 0, 1,# 54 is the number of frames. it should not be larger than 127
   9,  49,  53,  45,  24,  20,  -2,  15,
   8,  50,  41,  46,  28,  21,  -1,  15,
  10,  51,  26,  47,  26,  22,   6,  16,
  12,  52,  23,  48,  24,  24,   9,  17,
  14,  52,  20,  49,  22,  26,  12,  18,
  16,  53,  17,  51,  21,  27,  17,  18,
  18,  53,  14,  52,  20,  29,  22,  19,
  21,  54,  11,  54,  18,  30,  27,  19,
  22,  54,  11,  54,  18,  32,  29,  20,
  25,  54,  13,  55,  16,  34,  27,  21,
  26,  54,  16,  56,  16,  37,  24,  23,
  28,  54,  18,  56,  15,  39,  23,  24,
  30,  52,  20,  57,  14,  45,  22,  26,
  32,  54,  22,  57,  14,  44,  21,  28,
  33,  58,  24,  57,  15,  36,  20,  30,
  34,  61,  26,  57,  15,  31,  19,  32,
  36,  64,  28,  57,  14,  24,  18,  35,
  38,  66,  29,  57,  14,  20,  18,  38,
  39,  67,  31,  57,  14,  16,  17,  40,
  41,  64,  32,  56,  14,   5,  17,  43,
  42,  55,  35,  57,  14,  -1,  16,  44,
  44,  44,  37,  62,  15,  -3,  14,  35,
  45,  30,  39,  66,  15,   1,  14,  29,
  46,  21,  40,  68,  15,   5,  14,  23,
  47,  19,  42,  70,  16,   9,  14,  19,
  48,  16,  43,  70,  17,  12,  15,  17,
  49,  12,  44,  67,  18,  17,  15,   5,
  49,   9,  46,  59,  20,  24,  15,  -2,
  50,   8,  47,  47,  21,  28,  16,  -2,
  51,  10,  48,  34,  22,  26,  16,   1,
  52,  12,  49,  24,  24,  24,  17,   6,
  52,  14,  50,  21,  26,  22,  18,  10,
  53,  16,  51,  19,  27,  21,  19,  12,
  53,  18,  52,  15,  29,  20,  20,  19,
  54,  21,  54,  12,  30,  18,  19,  24,
  54,  22,  55,  12,  32,  18,  20,  27,
  54,  25,  55,  11,  34,  16,  22,  29,
  54,  26,  56,  14,  37,  16,  24,  26,
  54,  28,  56,  17,  39,  15,  25,  24,
  52,  30,  57,  18,  45,  14,  27,  23,
  54,  32,  57,  21,  44,  14,  29,  21,
  58,  33,  57,  23,  36,  15,  31,  20,
  61,  34,  57,  24,  31,  15,  33,  20,
  64,  36,  57,  26,  24,  14,  36,  19,
  66,  38,  57,  28,  20,  14,  39,  18,
  67,  39,  56,  30,  16,  14,  42,  17,
  64,  41,  56,  32,   5,  14,  45,  17,
  55,  42,  59,  33,  -1,  14,  41,  17,
  44,  44,  64,  35,  -3,  15,  33,  16,
  30,  45,  67,  38,   1,  15,  27,  14,
  21,  46,  68,  39,   5,  15,  22,  14,
  19,  47,  70,  41,   9,  16,  18,  14,
  16,  48,  69,  42,  12,  17,  11,  14,
  12,  49,  63,  44,  17,  18,   1,  15]

num_frames = wkF[0]
frame_data = np.array(wkF[4:]).reshape((num_frames, 8))

def simulate_bittle_gait(gait_frames, urdf_path):
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1 / 240)
    p.loadURDF("plane.urdf")

    robot = p.loadURDF(urdf_path, [0, 0, 0.1], flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT)

    joint_name_to_id = {}
    for i in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, i)
        joint_name = info[1].decode('utf-8')
        joint_name_to_id[joint_name] = i

    controlled_joint_names = [
        'right-front-shoulder-joint', 'right-front-knee-joint',
        'left-front-shoulder-joint', 'left-front-knee-joint',
        'right-back-shoulder-joint', 'right-back-knee-joint',
        'left-back-shoulder-joint', 'left-back-knee-joint'
    ]

    joint_ids = [joint_name_to_id[name] for name in controlled_joint_names]

    duration = 5.0  # seconds
    start_time = time.time()
    frame_interval = duration / num_frames  # time per frame
    frame_index = 0
    last_frame_time = start_time

    while time.time() - start_time < duration:
        now = time.time()
        if now - last_frame_time >= frame_interval:
            angles = gait_frames[frame_index % num_frames]
            for joint_id, angle in zip(joint_ids, angles):
                rad = np.radians(angle)
                p.setJointMotorControl2(robot, joint_id, p.POSITION_CONTROL, targetPosition=rad, force=2.5)
            frame_index += 1
            last_frame_time = now

        p.stepSimulation()
        time.sleep(1 / 240)

    p.disconnect()


if __name__ == "__main__":
    urdf_path = r"C:/Users/vansh/OneDrive/Documents/Engage Scholarship/Bittle_PyBullet-main/bittle2.urdf"
    simulate_bittle_gait(frame_data, urdf_path)
