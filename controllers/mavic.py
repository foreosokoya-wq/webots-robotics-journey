from controller import Robot

# Initialize the Robot instance
robot = Robot()
time_step = int(robot.getBasicTimeStep())

# 1. Initialize Sensors (IMU and Gyroscope)
imu = robot.getDevice("inertial unit")
imu.enable(time_step)

gyro = robot.getDevice("gyro")
gyro.enable(time_step)

# 2. Initialize the 4 Motors
motors = [
    robot.getDevice("front_left_motor"),
    robot.getDevice("front_right_motor"),
    robot.getDevice("rear_left_motor"),
    robot.getDevice("rear_right_motor")
]

# Set motors to velocity control mode (required for drones)
for motor in motors:
    motor.setPosition(float('inf'))
    motor.setVelocity(0.0)

# Main simulation loop
while robot.step(time_step) != -1:
    # Read sensor data
    roll, pitch, yaw = imu.getRollPitchYaw()
    wx, wy, wz = gyro.getValues()
    
    # Example: Apply a base velocity to lift off
    base_speed = 6.0
    for motor in motors:
        motor.setVelocity(base_speed)
