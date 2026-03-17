import math
import os
import numpy as np

def sysCall_init():
    sim = require('sim')
    # Update deprecated function calls
    self.bot = sim.getObject('/body')  # Changed from getObjectHandle
    self.r_motor = sim.getObject('/right_joint')
    self.l_motor = sim.getObject('/left_joint')
    self.prismatic_joint = sim.getObject('/Prismatic_joint')  # Define prismatic joint

    self.arm_joint = sim.getObject('/arm_joint')  # Define arm joint
    
    # PID gains for tilt and position
    self.kp_tilt = 10
    self.ki_tilt = 40
    self.kd_tilt = 0
    self.kp_pos = -10
    self.ki_pos = -0.1
    self.kd_pos = -20
    self.kp_heading = -1
    self.ki_heading = -0.001
    self.kd_heading = -0.1

    self.desired_tilt_angle = 0
    self.desired_position = sim.getObjectPosition(self.bot, -1)[1]
    self.desired_heading = 0  # Desired heading angle in degrees

    # PID state variables
    self.prev_tilt_error = 0
    self.integral_tilt = 0
    self.prev_position_error = 0
    self.integral_position = 0
    self.tilt_angle = 0
    self.current_position = 0
    self.last_time = sim.getSimulationTime()
    self.prev_heading_error = 0
    self.integral_heading = 0

    # Add variable for tilt velocity
    self.prev_tilt_angle = 0
    self.tilt_velocity = 0

    # Initialize lists for logging data
    self.time_history = []
    self.tilt_history = []
    self.position_history = []
    self.control_signal_history = []
    self.feedforward_history = []

def sysCall_actuation():
    
    message, data, data2 = sim.getSimulatorMessage()    
    if message == sim.message_keypress:
        if data[0] == 2007:  # Up arrow key
            self.desired_position += 0.2  # Increase position change per press (adjust for faster response)
        elif data[0] == 2008:  # Down arrow key
            self.desired_position -= 0.2  # Increase position change per press (adjust for faster response)
        elif data[0] == 2009:  # Left arrow key
            self.desired_heading -= 80  # Increase heading change per press (adjust for faster response)
        
        elif data[0] == 2010:  # Right arrow key
            self.desired_heading += 80  # Increase heading change per press (adjust for faster response)
        
        # Set 'Prismatic_joint' velocity
        if data[0] == 113:  # q key
            sim.setJointTargetVelocity(self.prismatic_joint, 0.1)
            
        elif data[0] == 101:  # e key
            sim.setJointTargetVelocity(self.prismatic_joint, -0.1)

        # Set 'arm_joint' velocity
        elif data[0] == 119:  # w key
            sim.setJointTargetVelocity(self.arm_joint, 1)
        elif data[0] == 115:  # s key
            sim.setJointTargetVelocity(self.arm_joint, -1)
        
        elif data[0] == 120: #x key
            sim.setJointTargetVelocity(self.arm_joint, 0)
        elif data[0] == 121: #y key
            sim.setJointTargetVelocity(self.prismatic_joint, 0)
        elif data[0] == 122: #z key
            sim.setJointTargetVelocity(self.l_motor, 0)
            sim.setJointTargetVelocity(self.r_motor, 0)
    

    current_time = sim.getSimulationTime()
    delta_time = (current_time - self.last_time)

    if delta_time > 0:
        # Calculate tilt velocity
        self.tilt_velocity = (self.tilt_angle - self.prev_tilt_angle) / delta_time
        
        # Heading Control (Outer Loop)
        heading_error = self.desired_heading - self.current_heading
        self.integral_heading += heading_error * delta_time
        derivative_heading = (heading_error - self.prev_heading_error) / delta_time

        PID_heading_control = (self.kp_heading * heading_error +
                               self.ki_heading * self.integral_heading +
                               self.kd_heading * derivative_heading)
        PID_heading_control = np.clip(PID_heading_control, -1, 1)
        
        # Position Control (Outer Loop)
        position_error = self.desired_position - self.current_position
        self.integral_position += position_error * delta_time
        derivative_position = (position_error - self.prev_position_error) / delta_time
        PID_position_control = (self.kp_pos * position_error +
                                self.ki_pos * self.integral_position +
                                self.kd_pos * derivative_position)
        
        # Tilt Control (Inner Loop)
        dither_amplitude = 1  # Amplitude of the dithering signal
        dither_frequency = 2.0  # Frequency of the dithering signal in Hz
        dither_signal = dither_amplitude * np.sin(2 * np.pi * dither_frequency * current_time)
        theta_ref = PID_position_control
        theta_ref = np.clip(PID_position_control, -5, 5) + 0.2 * np.sign(position_error)
        tilt_error = theta_ref - self.tilt_angle
        self.integral_tilt += tilt_error * delta_time
        derivative_tilt = (tilt_error - self.prev_tilt_error) / delta_time
        PID_tilt_control = (self.kp_tilt * tilt_error +
                            self.ki_tilt * self.integral_tilt +
                            self.kd_tilt * derivative_tilt)
        
        # Calculate feed-forward term
        linear_velocity, angular_velocity = sim.getObjectVelocity(self.bot)
        linear_velocity = np.array(linear_velocity)
        
        feedforward = (-52.10) * np.linalg.norm(linear_velocity) * np.sign(theta_ref)
        # Combine PID and feed-forward
        control_signal = 1 * PID_tilt_control + feedforward
        print(current_time, self.desired_heading, heading_error, PID_heading_control,
              self.desired_position, theta_ref, PID_heading_control, position_error,
              PID_position_control)
        
        # Apply control to both motors
        sim.setJointTargetVelocity(self.l_motor, control_signal - PID_heading_control)
        sim.setJointTargetVelocity(self.r_motor, control_signal + PID_heading_control)
        # Handle keyboard inputs to adjust desired_position and desired_heading
        
    
        
        # Log data
        self.time_history.append(current_time)
        self.tilt_history.append(self.tilt_angle)
        self.position_history.append(self.current_position)
        self.control_signal_history.append(control_signal)
        self.feedforward_history.append(feedforward)
        
        # Update previous values
        self.prev_tilt_error = tilt_error
        self.prev_position_error = position_error
        self.prev_tilt_angle = self.tilt_angle
        self.last_time = current_time

def sysCall_sensing():
    # Convert tilt angle from radians to degrees
    self.tilt_angle = math.degrees(sim.getObjectOrientation(self.bot, -1)[0])
    self.current_position = sim.getObjectPosition(self.bot, -1)[1]
    self.current_heading = math.degrees(sim.getObjectOrientation(self.bot, -1)[2])

def sysCall_cleanup():
    # Create directory if it doesn't exist
    log_dir = os.path.dirname(r'C:\Users\User\Desktop\Task1B_windows\Task1B_windows')
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
        
    # Write the logged data to file
    try:
        with open(r'C:\Users\User\Desktop\Task1B_windows\Task1B_windows\log_data.csv', 'w') as f:
            f.write('time,tilt_angle,position,control_signal,feedforward\n')
            for i in range(len(self.time_history)):
                f.write(f'{self.time_history[i]},{self.tilt_history[i]},{self.position_history[i]},{self.control_signal_history[i]},{self.feedforward_history[i]}\n')
    except Exception as e:
        print(f"Error writing to file: {e}")