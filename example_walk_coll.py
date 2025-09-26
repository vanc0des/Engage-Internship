import pybullet as p
import pybullet_data
import time
import math


def init_sim(urdf_path):
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -1.0)
    p.setTimeStep(1 / 240)
    p.loadURDF("plane.urdf")
    robot = p.loadURDF(urdf_path, [0, 0, 5], flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT)

    p.changeVisualShape(robot, -1, rgbaColor=[1, 1, 1, 1])
    for i in range(p.getNumJoints(robot)):
        p.changeVisualShape(robot, i, rgbaColor=[0.5, 0.5, 1, 1])

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

    joint_offsets = {
        'right-front-knee-joint': -0.7,
        'right-back-knee-joint': -0.7,
        'left-front-knee-joint': 0.7,
        'left-back-knee-joint': 0.7,
        'right-front-shoulder-joint': -0.2,
        'right-back-shoulder-joint': -0.2,
        'left-front-shoulder-joint': 0.2,
        'left-back-shoulder-joint': 0.2
    }

    for _ in range(int(3.0 * 240)):
        for name in controlled_joint_names:
            joint_id = joint_name_to_id[name]
            offset = joint_offsets.get(name, 0.0)
            p.resetJointState(robot, joint_id, targetValue=offset, targetVelocity=0)
        p.stepSimulation()
        time.sleep(1 / 240)

    p.setGravity(0, 0, -9.81)
    state_id = p.saveState()
    return robot, joint_name_to_id, joint_offsets, state_id


def setup_sliders():
    sliders = {
        'frequency': p.addUserDebugParameter('Frequency (Hz)', 0.1, 5.0, 1.0),
        'amplitude_shoulder': p.addUserDebugParameter('Amplitude Shoulder (rad)', 0.0, 1.0, 0.4),
        'amplitude_knee': p.addUserDebugParameter('Amplitude Knee (rad)', 0.0, 1.0, 0.2),
        'phase_shoulder_knee': p.addUserDebugParameter('Phase (Shoulder-Knee) [rad]', 0.0, math.pi, math.pi),
        'phase_front_back': p.addUserDebugParameter('Phase (Front-Back Legs) [rad]', 0.0, math.pi, 0.0),
        'phase_left_right': p.addUserDebugParameter('Phase (Left-Right Legs) [rad]', 0.0, math.pi, 0.0),  # Fixed syntax
        'start_button': p.addUserDebugParameter('Start Gait (Push)', 0, 1, 0),
        'reset_button': p.addUserDebugParameter('Reset Robot (Push)', 0, 1, 0),
        'speed_measurement': p.addUserDebugParameter('Measure Speed (5s)', 0, 1, 0),
        'reset_speed': p.addUserDebugParameter('Reset Speed Display', 0, 1, 0)
    }
    return sliders


def run_gait(robot, joint_name_to_id, joint_offsets, sliders, state_id):
    shoulder_joints = {name for name in joint_name_to_id if 'shoulder' in name}
    knee_joints = {name for name in joint_name_to_id if 'knee' in name}

    left_knees_front = []
    left_knees_back = []
    right_knees_front = []
    right_knees_back = []
    base_link = -1

    for i in range(p.getNumJoints(robot)):
        link_name = p.getJointInfo(robot, i)[12].decode('utf-8')
        if "knee" in link_name:
            if "left" in link_name:
                if "front" in link_name:
                    left_knees_front.append(i)
                elif "back" in link_name:
                    left_knees_back.append(i)
            elif "right" in link_name:
                if "front" in link_name:
                    right_knees_front.append(i)
                elif "back" in link_name:
                    right_knees_back.append(i)

    start_time = time.time()
    gait_active = False
    prev_start_val = 0
    prev_reset_val = 0
    prev_speed_val = 0
    prev_reset_speed_val = 0
    last_collision_time = 0
    highlighted_links = set()
    
    # Speed measurement variables
    speed_measuring = False
    speed_start_time = 0
    speed_start_pos = None
    current_speed = 0.0

    while True:
        t = time.time() - start_time

        frequency = p.readUserDebugParameter(sliders['frequency'])
        amplitude_shoulder = p.readUserDebugParameter(sliders['amplitude_shoulder'])
        amplitude_knee = p.readUserDebugParameter(sliders['amplitude_knee'])
        phase_sk = p.readUserDebugParameter(sliders['phase_shoulder_knee'])
        phase_fb = p.readUserDebugParameter(sliders['phase_front_back'])
        phase_lr = p.readUserDebugParameter(sliders['phase_left_right'])  # Now using left-right phase

        current_start_val = p.readUserDebugParameter(sliders['start_button'])
        current_reset_val = p.readUserDebugParameter(sliders['reset_button'])
        current_speed_val = p.readUserDebugParameter(sliders['speed_measurement'])
        current_reset_speed_val = p.readUserDebugParameter(sliders['reset_speed'])

        # Toggle gait
        if prev_start_val <= 0.5 and current_start_val > 0.5:
            gait_active = not gait_active
            print(f"[INFO] Gait toggled {'ON' if gait_active else 'OFF'}")
        prev_start_val = current_start_val

        # Reset robot
        if prev_reset_val <= 0.5 and current_reset_val > 0.5:
            p.restoreState(state_id)
            start_time = time.time()
            gait_active = False
            prev_start_val = 0
            speed_measuring = False
            current_speed = 0.0
            print("[INFO] Robot reset and gait deactivated.")
            for name, joint_id in joint_name_to_id.items():
                offset = joint_offsets.get(name, 0.0)
                p.resetJointState(robot, joint_id, targetValue=offset, targetVelocity=0)
        prev_reset_val = current_reset_val

        # Speed measurement
        if prev_speed_val <= 0.5 and current_speed_val > 0.5:
            if not speed_measuring:
                # Start speed measurement
                robot_pos, _ = p.getBasePositionAndOrientation(robot)
                speed_start_pos = robot_pos
                speed_start_time = time.time()
                speed_measuring = True
                print("[INFO] Starting speed measurement (5 seconds)...")
            else:
                print("[INFO] Speed measurement already in progress...")
        prev_speed_val = current_speed_val

        # Reset speed display
        if prev_reset_speed_val <= 0.5 and current_reset_speed_val > 0.5:
            current_speed = 0.0
            speed_measuring = False
            p.removeAllUserDebugItems()  # Clear speed display
            print("[INFO] Speed display reset.")
        prev_reset_speed_val = current_reset_speed_val

        # Check if speed measurement is complete
        if speed_measuring and (time.time() - speed_start_time) >= 5.0:
            robot_pos, _ = p.getBasePositionAndOrientation(robot)
            # Calculate distance traveled using vector magnitude
            dx = robot_pos[0] - speed_start_pos[0]
            dy = robot_pos[1] - speed_start_pos[1]
            dz = robot_pos[2] - speed_start_pos[2]
            distance = math.sqrt(dx**2 + dy**2 + dz**2)
            current_speed = distance / 5.0  # m/s
            speed_measuring = False
            print(f"[SPEED] Distance traveled: {distance:.3f} m")
            print(f"[SPEED] Average speed: {current_speed:.3f} m/s")

        # Display current speed if measured (update less frequently)
        if current_speed > 0 and int(t * 10) % 10 == 0:  # Update every 0.1 seconds
            p.removeAllUserDebugItems()  # Clear previous text
            p.addUserDebugText(f"Speed: {current_speed:.3f} m/s", [0.5, 0, 1], 
                             textColorRGB=[1, 1, 0], textSize=1.5)

        # Visual feedback for ongoing speed measurement (update less frequently)
        if speed_measuring and int(t * 10) % 5 == 0:  # Update every 0.5 seconds
            p.removeAllUserDebugItems()  # Clear previous text
            elapsed = time.time() - speed_start_time
            remaining = 5.0 - elapsed
            p.addUserDebugText(f"Measuring... {remaining:.1f}s left", [0.5, 0, 1.5], 
                             textColorRGB=[0, 1, 0], textSize=1.2)

        # Reset visual highlights
        for link_id in highlighted_links:
            p.changeVisualShape(robot, link_id, rgbaColor=[0.5, 0.5, 1.0, 1.0])
        highlighted_links.clear()

        # Joint control with left-right phase
        for name, joint_id in joint_name_to_id.items():
            base = joint_offsets.get(name, 0.0)
            if gait_active:
                is_left = 'left' in name
                is_front = 'front' in name
                
                # Calculate phases
                fb_phase = 0 if is_front else phase_fb
                lr_phase = phase_lr if is_left else 0  # Apply left-right phase to left legs
                
                total_phase = 2 * math.pi * frequency * t + fb_phase + lr_phase

                if name in shoulder_joints:
                    sign = 1 if is_left else -1
                    target = base + sign * amplitude_shoulder * math.sin(total_phase)
                elif name in knee_joints:
                    sign = 1 if is_left else -1
                    target = base + sign * amplitude_knee * math.sin(total_phase + phase_sk)
                else:
                    continue
            else:
                target = base

            try:
                p.setJointMotorControl2(robot, joint_id, p.POSITION_CONTROL, targetPosition=target, force=2.5)
            except p.error:
                print("[ERROR] Connection to physics server lost. Reconnecting...")
                return  # Exit gracefully

        # Collision detection (check less frequently for better performance)
        if int(t * 60) % 6 == 0:  # Check every 0.1 seconds instead of every frame
            for lk in left_knees_front + left_knees_back:
                pts = p.getClosestPoints(robot, robot, distance=0.01, linkIndexA=lk, linkIndexB=base_link)
                if pts:
                    if time.time() - last_collision_time > 0.5:
                        print(f"[Collision] Left knee {lk} ↔ Base")
                        last_collision_time = time.time()
                    p.changeVisualShape(robot, lk, rgbaColor=[1, 1, 0, 1])
                    p.changeVisualShape(robot, base_link, rgbaColor=[1, 1, 0, 1])
                    highlighted_links.update([lk, base_link])

            for rk in right_knees_front + right_knees_back:
                pts = p.getClosestPoints(robot, robot, distance=0.01, linkIndexA=rk, linkIndexB=base_link)
                if pts:
                    if time.time() - last_collision_time > 0.5:
                        print(f"[Collision] Right knee {rk} ↔ Base")
                        last_collision_time = time.time()
                    p.changeVisualShape(robot, rk, rgbaColor=[1, 0.5, 0, 1])
                    p.changeVisualShape(robot, base_link, rgbaColor=[1, 0.5, 0, 1])
                    highlighted_links.update([rk, base_link])

            # Only check front-front and back-back cross-side collisions
            for lf, rf in zip(left_knees_front, right_knees_front):
                pts = p.getClosestPoints(robot, robot, distance=0.01, linkIndexA=lf, linkIndexB=rf)
                if pts:
                    if time.time() - last_collision_time > 0.5:
                        print(f"[Collision] Left front {lf} ↔ Right front {rf}")
                        last_collision_time = time.time()
                    p.changeVisualShape(robot, lf, rgbaColor=[1, 0, 0, 1])
                    p.changeVisualShape(robot, rf, rgbaColor=[1, 0, 0, 1])
                    highlighted_links.update([lf, rf])

            for lb, rb in zip(left_knees_back, right_knees_back):
                pts = p.getClosestPoints(robot, robot, distance=0.01, linkIndexA=lb, linkIndexB=rb)
                if pts:
                    if time.time() - last_collision_time > 0.5:
                        print(f"[Collision] Left back {lb} ↔ Right back {rb}")
                        last_collision_time = time.time()
                    p.changeVisualShape(robot, lb, rgbaColor=[1, 0, 0, 1])
                    p.changeVisualShape(robot, rb, rgbaColor=[1, 0, 0, 1])
                    highlighted_links.update([lb, rb])

        p.stepSimulation()
        time.sleep(1 / 240)


def main():
    urdf_path = r"C:/Users/vansh/OneDrive/Documents/Engage Scholarship/Bittle_PyBullet-main/bittle2.urdf"
    robot, joint_name_to_id, joint_offsets, state_id = init_sim(urdf_path)
    sliders = setup_sliders()
    run_gait(robot, joint_name_to_id, joint_offsets, sliders, state_id)


if __name__ == "__main__":
    main()
