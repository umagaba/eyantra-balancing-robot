import math

def sysCall_init():
    sim = require('sim')

    self.bot = sim.getObjectHandle('body')
    self.r_motor = sim.getObjectHandle('right_joint')
    self.l_motor = sim.getObjectHandle('left_joint')

    # PID gains for tilt and position
    self.kp_tilt = 23
    self.ki_tilt = 0.1
    self.kd_tilt = 0.01
    self.kp_pos = 1
    self.ki_pos = 0
    self.kd_pos = -10

    self.desired_tilt_angle = 0  # Desired tilt angle in degrees
    self.desired_position = sim.getObjectPosition(self.bot, -1)[1]

    # PID state variables
    self.prev_tilt_error = 0
    self.integral_tilt = 0
    self.prev_position_error = 0
    self.integral_position = 0
    self.tilt_angle = 0
    self.current_position = 0
    self.last_time = sim.getSimulationTime()

    # Initialize lists for logging data
    self.time_history = []
    self.tilt_history = []
    self.position_history = []
    self.control_signal_history = []

def sysCall_actuation():
    current_time = sim.getSimulationTime()
    delta_time = (current_time - self.last_time)

    if delta_time > 0:
        
        # Position Control (Outer Loop)
        position_error = self.desired_position - self.current_position
        self.integral_position += position_error * delta_time
        derivative_position = (position_error - self.prev_position_error) / delta_time

        PID_position_control = (self.kp_pos * position_error +
                                self.ki_pos * self.integral_position +
                                self.kd_pos * derivative_position)
                                
        # Tilt Control (Inner Loop)
        theta_ref = PID_position_control
        tilt_error = theta_ref - self.tilt_angle
        self.integral_tilt += tilt_error * delta_time
        derivative_tilt = (tilt_error - self.prev_tilt_error) / delta_time

        PID_tilt_control = (self.kp_tilt * tilt_error +
                            self.ki_tilt * self.integral_tilt +
                            self.kd_tilt * derivative_tilt)

        

        # Combine the control signals
        control_signal = PID_tilt_control 

        # Apply control to both motors
        sim.setJointTargetVelocity(self.l_motor, 1*control_signal)
        sim.setJointTargetVelocity(self.r_motor, 1*control_signal)

        # Log the time, tilt, position, and control signal for plotting
        self.time_history.append(current_time)
        self.tilt_history.append(self.tilt_angle)
        self.position_history.append(self.current_position)
        self.control_signal_history.append(control_signal)
        print(theta_ref)
        # Update previous errors and time
        self.prev_tilt_error = tilt_error
        self.prev_position_error = position_error
        self.last_time = current_time

def sysCall_sensing():
    # Convert tilt angle from radians to degrees
    self.tilt_angle = math.degrees(sim.getObjectOrientation(self.bot, -1)[0])
    self.current_position = sim.getObjectPosition(self.bot, -1)[1]

def sysCall_cleanup():
    # Write the logged data to a file at the end of the simulation
    with open(r'C:\Users\User\Desktop\Task1B_windows\Task1B_windows\log_data.csv', 'w') as f:
        f.write('time,tilt_angle,position,control_signal\n')
        for i in range(len(self.time_history)):
            f.write(f'{self.time_history[i]},{self.tilt_history[i]},{self.position_history[i]},{self.control_signal_history[i]}\n')