"""e_puck_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, DistanceSensor

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#target = 75
base_speed = 3.0
turn_speed = 2.0
Kp = 0.002 # 0.5
Ki = 0.000001 # 0.001
Kd = 0.00002 # 0.01
integral_error = 0
previous_error = 0

wheel_radius = 0.0205
wheel_distance = 0.052
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

left_encoder = robot.getDevice("left wheel sensor")
right_encoder = robot.getDevice("right wheel sensor")

left_encoder.enable(timestep)
right_encoder.enable(timestep)

robot.step(timestep)

initial_left = left_encoder.getValue()
initial_right = right_encoder.getValue()

#  motor = robot.getDevice('motorname')
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
caution_threshold = 95
danger_threshold = 150
sensors = []
for i in range(8):
    sensor = robot.getDevice(f"ps{i}")
    sensor.enable(timestep)
    sensors.append(sensor)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    obstacles = []
    values = [sensor.getValue() for sensor in sensors]
    for value in values:
        if value > caution_threshold:
            obstacles.append(True)
        else:
            obstacles.append(False)
    for i, obstacle in enumerate(obstacles):
        if obstacle:
            print(f"ps{i}: OBSTACLE")
        else:
            print(f"ps{i}: CLEAR")
    #print(obstacles)
    front = (values[0] + values[7]) / 2
    right = (values[1] + values[2]) / 2
    back = (values[3] + values[4]) / 2
    left = (values[5] + values[6]) / 2
    
    left_position = left_encoder.getValue()
    right_position = right_encoder.getValue()
    
    previous_left = initial_left
    previous_right = initial_right
    
    left_change = left_position - initial_left
    right_change = right_position - initial_right
    
    left_distance = left_change * wheel_radius
    right_distance = right_change * wheel_radius
    
    delta_change = (right_distance - left_distance) / wheel_distance
    
    distance = (left_distance + right_distance) / 2
    
    previous_left = left_position
    previous_right = right_position
    
    directions = {
    "front": front,
    "right": right,
    "back": back,
    "left": left
    }
# 1. Calculate PID Correction ONCE per loop
    error = right - left
    P = Kp * error
    
    integral_error += error
    integral_error = max(-1000, min(1000, integral_error))
    I = Ki * integral_error
    
    dt = timestep / 1000.0
    derivative = (error - previous_error) / dt
    D = Kd * derivative
    
    correction = P + I + D
    previous_error = error 
    
    # 2. Decision Logic
    if front > danger_threshold:
        integral_error = 0 # Reset windup on emergency turn
        
        if left > danger_threshold and right > danger_threshold:
            if back < danger_threshold: # Fixed spelling
                left_speed = -base_speed
                right_speed = -base_speed # Fixed minus to equals
            else:
                if left < right: 
                    left_speed = -turn_speed
                    right_speed = turn_speed
                else:
                    left_speed = turn_speed
                    right_speed = -turn_speed
        
        elif left < right:
            left_speed = -turn_speed
            right_speed = turn_speed
            
        else:
            left_speed = turn_speed
            right_speed = -turn_speed
            
    elif front > caution_threshold:
        # Slow down, but still use the PID correction to balance
        base_caution_speed = 1.5
        left_speed = base_caution_speed - correction
        right_speed = base_caution_speed + correction
        
    else:
        # Open space, move fast with PID balancing
        left_speed = base_speed - correction
        right_speed = base_speed + correction
        
    
    # 3. Apply Speeds
    left_speed =  max(-6.28, min(6.28, left_speed))
    right_speed = max(-6.28, min(6.28, right_speed))
        
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)    
    
    most_blocked = max(directions, key = directions.get)
    most_open = min(directions, key = directions.get)
    print(
    f"Front: {front:.1f} | "
    f"Right: {right:.1f} | "
    f"Back: {back:.1f} | "
    f"Left: {left:.1f}"
    )

    print(
        f"Most blocked: {most_blocked} | "
        f"Most open: {most_open} | "
        f"Error: {error} | "
        f"Correction: {correction}"
    )
    print(
        f"Left: {left_distance:.3f} m | "
        f"Right: {right_distance:.3f} m | "
        f"Robot: {distance:.3f} m "
    )
 
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    

# Enter here exit cleanup code.
