#!/usr/bin/python3
# -*- coding: UTF-8 -*-

from PetoiRobot import *
import math
import time
import threading

class SimpleGaitController:
    def __init__(self):
        """Initialize the gait controller with joint mappings."""
        
        # Joint mapping from simulation names to Petoi robot indices
        # Adjust these indices based on your specific robot model
        self.joint_mapping = {
            'right-front-shoulder-joint': 8,
            'right-front-knee-joint': 12,
            'left-front-shoulder-joint': 9,
            'left-front-knee-joint': 13,
            'right-back-shoulder-joint': 10,
            'right-back-knee-joint': 14,
            'left-back-shoulder-joint': 11,
            'left-back-knee-joint': 15
        }
        
        # Joint offsets (neutral positions in degrees)
        self.joint_offsets = {
            'right-front-knee-joint': 40,
            'right-back-knee-joint': 40,
            'left-front-knee-joint': 40,
            'left-back-knee-joint': 40,
            'right-front-shoulder-joint': -11,
            'right-back-shoulder-joint': -11,
            'left-front-shoulder-joint': 11,
            'left-back-shoulder-joint': 11
        }
        
        # Control variables
        self.gait_active = False
        self.stop_gait = False
        self.gait_thread = None
        
    def calculate_joint_angle(self, joint_name, time_elapsed, gait_params):
        """Calculate joint angle using sine wave based on gait parameters."""
        
        # Get base offset
        base_angle = self.joint_offsets.get(joint_name, 0)
        
        # Joint characteristics
        is_left = 'left' in joint_name
        is_front = 'front' in joint_name
        is_shoulder = 'shoulder' in joint_name
        is_knee = 'knee' in joint_name
        
        # Phase calculations
        # For knees, reverse direction
        if is_knee:
            side_sign = -1 if is_left else 1
        else:
            side_sign = 1 if is_left else -1

        front_back_phase = 0 if is_front else math.radians(gait_params['phase_front_back'])
        time_phase = 2 * math.pi * gait_params['frequency'] * time_elapsed + front_back_phase
        
        # Amplitude and phase based on joint type
        if is_shoulder:
            amplitude = gait_params['amplitude_shoulder']
            additional_phase = 0
        elif is_knee:
            amplitude = gait_params['amplitude_knee']
            additional_phase = math.radians(gait_params['phase_shoulder_knee'])
        else:
            return base_angle
            
        # Calculate final angle
        sine_component = side_sign * amplitude * math.sin(time_phase + additional_phase)
        target_angle = base_angle + sine_component
        
        # Limit servo range
        target_angle = max(-125, min(125, target_angle))
        
        return target_angle
        
    def gait_loop(self, gait_params):
        """Main gait control loop."""
        
        start_time = time.time()
        update_rate = 50  # Hz
        sleep_time = 1.0 / update_rate
        
        print(f"Walking with parameters: {gait_params}")
        
        while not self.stop_gait:
            elapsed_time = time.time() - start_time
            
            # Calculate joint angles
            joint_commands = []
            for joint_name, joint_index in self.joint_mapping.items():
                target_angle = self.calculate_joint_angle(joint_name, elapsed_time, gait_params)
                joint_commands.extend([joint_index, int(target_angle)])
                
            # Send to robot
            try:
                send(goodPorts, ['I', joint_commands, 0])
            except Exception as e:
                print(f"Error sending commands: {e}")
                break
                
            time.sleep(sleep_time)
            
    def walk_with_gait(self, frequency, amplitude_shoulder, amplitude_knee, 
                      phase_shoulder_knee, phase_front_back, duration=10):
        """
        Make the robot walk with specified gait parameters.
        
        Parameters:
        - frequency: Gait frequency in Hz
        - amplitude_shoulder: Shoulder joint amplitude in degrees
        - amplitude_knee: Knee joint amplitude in degrees  
        - phase_shoulder_knee: Phase difference between shoulder and knee in degrees
        - phase_front_back: Phase difference between front and back legs in degrees
        - duration: How long to walk in seconds
        """
        
        if self.gait_active:
            print("Gait already active. Stop current gait first.")
            return
            
        gait_params = {
            'frequency': frequency,
            'amplitude_shoulder': amplitude_shoulder,
            'amplitude_knee': amplitude_knee,
            'phase_shoulder_knee': phase_shoulder_knee,
            'phase_front_back': phase_front_back
        }
        
        self.gait_active = True
        self.stop_gait = False    
        # Start gait thread
        self.gait_thread = threading.Thread(
            target=self.gait_loop, 
            args=(gait_params,), 
            daemon=True
        )
        self.gait_thread.start()
        
        # Walk for specified duration
        time.sleep(duration)
        
        # Stop walking
        self.stop_walking()
        
    def stop_walking(self):
        """Stop the gait and return to neutral position."""
        
        self.stop_gait = True
        self.gait_active = False
        
        # Wait for thread to finish
        if self.gait_thread and self.gait_thread.is_alive():
            self.gait_thread.join(timeout=1.0)
            
        # Return to neutral positions
        neutral_commands = []
        for joint_name, joint_index in self.joint_mapping.items():
            neutral_angle = self.joint_offsets.get(joint_name, 0)
            neutral_commands.extend([joint_index, int(neutral_angle)])
            
        try:
            send(goodPorts, ['I', neutral_commands, 0])
            print("Returned to neutral position")
        except Exception as e:
            print(f"Error returning to neutral: {e}")


def make_robot_walk():
    """Simple function to make the robot walk with your recorded parameters."""
    
    try:
        # Connect to robot
        autoConnect()
        time.sleep(1)
        
        # Wake up and balance
        send(goodPorts, ['kbalance', 1])
        time.sleep(2)
        
        # Create controller
        controller = SimpleGaitController()
        
        # YOUR RECORDED GAIT PARAMETERS - Replace these with your values
        # Example values - replace with your recorded parameters from simulation
        
        print("=== Walking Gait 1 ===")
        controller.walk_with_gait(
            frequency=1.0,              # Replace with your recorded frequency
            amplitude_shoulder=20,      # Replace with your recorded shoulder amplitude  
            amplitude_knee=15,          # Replace with your recorded knee amplitude
            phase_shoulder_knee=180,    # Replace with your recorded shoulder-knee phase
            phase_front_back=0,         # Replace with your recorded front-back phase
            duration=5                  # Walk for 5 seconds
        )
        
        time.sleep(2)  # Pause between gaits
        
        print("=== Walking Gait 2 ===")
        controller.walk_with_gait(
            frequency=1.5,              # Different gait parameters
            amplitude_shoulder=25,      
            amplitude_knee=18,          
            phase_shoulder_knee=90,     
            phase_front_back=180,       
            duration=5
        )
        
        # Sit down when finished
        send(goodPorts, ['ksit', 2])
        print("Walking complete")
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        closePort()


def walk_with_custom_parameters(freq, amp_shoulder, amp_knee, phase_sk, phase_fb, duration=5):
    """
    Quick function to test specific parameter combinations.
    
    Usage example:
    walk_with_custom_parameters(1.2, 22, 16, 180, 0, 8)
    """
    
    try:
        autoConnect()
        time.sleep(1)
        send(goodPorts, ['kbalance', 1])
        time.sleep(2)
        
        controller = SimpleGaitController()
        controller.walk_with_gait(freq, amp_shoulder, amp_knee, phase_sk, phase_fb, duration)
        
        send(goodPorts, ['ksit', 1])
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        closePort()


if __name__ == '__main__':
    # Simply run this to make the robot walk
    # Edit the parameters in make_robot_walk() function above
    make_robot_walk()
    
    # Or use the custom function:
    # walk_with_custom_parameters(1.0, 20, 15, 180, 0, 10)
