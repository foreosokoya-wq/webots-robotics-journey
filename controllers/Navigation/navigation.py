import math
from controller import Robot, Motor, DistanceSensor
robot = Robot()
timestep = int(robot.getBasicTimeStep())
target = 90
base_speed = 2.0
turn_speed = 2.0
K_heading = 2.0
Kp = 0.05#0.005 # 0.5
Ki = 0.000001 # 0.001
Kd = 0.035#0.0008 # 0.01
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

previous_left = left_encoder.getValue()
previous_right = right_encoder.getValue()

#  motor = robot.getDevice('motorname')
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

x = 0.0
y = 0.0
theta = 0.0

target_x = 1.0
target_y = 1.0

#θ

#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
caution_threshold = 78
danger_threshold = 150
sensors = []
for i in range(8):
    sensor = robot.getDevice(f"ps{i}")
    sensor.enable(timestep)
    sensors.append(sensor)

while robot.step(timestep) != -1:

    
    # WAY POINT NAVIGATION
    dx = target_x - x
    dy = target_y - y
    
    distance_to_target = math.sqrt(dx**2 + dy**2)
    
    target_theta = math.atan2(dy, dx)
    heading_error = target_theta - theta
    heading_error = math.atan2(
        math.sin(heading_error),
        math.cos(heading_error)
        )
    
    
    left_position = left_encoder.getValue()
    right_position = right_encoder.getValue()

    left_change = left_position - previous_left
    right_change = right_position - previous_right
    
    left_distance = left_change * wheel_radius
    right_distance = right_change * wheel_radius
    
    delta_theta = (right_distance - left_distance) / wheel_distance
    #delta_theta_deg = (180 / math.pi) * delta_theta
    theta_mid = theta + delta_theta / 2
    
    distance = (left_distance + right_distance) / 2
    x += distance * math.cos(theta_mid)
    y += distance * math.sin(theta_mid)
    
    theta += delta_theta
    theta = math.atan2(math.sin(theta), math.cos(theta))
    
    previous_left = left_position
    previous_right = right_position
        
    tolerance = 0.05
    if distance_to_target < tolerance:
        left_speed = 0.0
        right_speed = 0.0
    else:
        # ADDING PROPORTIONAL FEEDBACK SYSTEM
        correction = K_heading * heading_error
        
        base_speed = min(2.0, distance_to_target * 2.0)
        alignment = math.cos(heading_error)
        base_speed *= max(0.0, alignment)
        
        
        left_speed = base_speed - correction
        right_speed = base_speed + correction
        
        left_speed = max(-6.28, min(6.28, left_speed))
        right_speed = max(-6.28, min(6.28, right_speed))
            
    
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    
    print(
        #f"Left Distance: {left_distance:.6f} | "
        #f"Right Distance: {right_distance:.6f} | "
        #f"X: {x:.3f} | "
        #f"Y: {y:.3f} | "
        f"θ: {theta:.3f} "
    )
    print(
        f"Position: ({x:.3f}, {y:.3f}) | "
        f"Target distance: {distance_to_target:.3f} | "
        f"Heading error: {math.degrees(heading_error):.1f}°"
    )