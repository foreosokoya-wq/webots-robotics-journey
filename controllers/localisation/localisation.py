import math
from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())


# Robot Parts geometry

wheel_radius = 0.0205
wheel_distance = 0.052


# Motors

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

left_motor.setVelocity(0)
right_motor.setVelocity(0)


# Encoders

left_encoder = robot.getDevice("left wheel sensor")
right_encoder = robot.getDevice("right wheel sensor")

left_encoder.enable(timestep)
right_encoder.enable(timestep)


# GPS

gps = robot.getDevice("gps")
gps.enable(timestep)

robot.step(timestep)

gps_start = gps.getValues()
gps_x0 = gps_start[0]
gps_y0 = gps_start[1]

# Initiate LIDAR
lidar = robot.getDevice("lidar")
lidar.enable(timestep)

resolution = lidar.getHorizontalResolution()
fov = lidar.getFov()

# Let sensors initialize
robot.step(timestep)

# Initial encoder readings
previous_left = left_encoder.getValue()
previous_right = right_encoder.getValue()

# Our local odometry pose
x = 0.0
y = 0.0
theta = 0.0

# Target pose
waypoints = [
    (1.5, 0.0),
    (0.5, 0.5),
    (0.0, 0.5),
    (0.0, 0.0)
]

current_waypoint = 0

# K_heading
K_heading = 2.0
base_speed = 3.0
tolerance = 0.07

# Obstacle Avoidance constants
obstacle_threshold = 0.20
avoidance_speed = 2.5

while robot.step(timestep) != -1:

    
    target_x, target_y = waypoints[current_waypoint]
    

    #  Read encoders


    current_left = left_encoder.getValue()
    current_right = right_encoder.getValue()

    left_change = current_left - previous_left
    right_change = current_right - previous_right

    left_distance = left_change * wheel_radius
    right_distance = right_change * wheel_radius

    previous_left = current_left
    previous_right = current_right
    

    #  Read GPS


    gps_position = gps.getValues()

    gps_x = gps_position[0]
    gps_y = gps_position[1]
    gps_z = gps_position[2]
    
    gps_local_x = gps_x - gps_x0
    gps_local_y = gps_y - gps_y0
    
    error_x = gps_local_x - x
    error_y = gps_local_y - y
    
    correction_gain = 0.2
    
   # x += correction_gain * error_x
   # y += correction_gain * error_y


    # Differential-drive odometry

    # Navigation
    dx = target_x - x
    dy = target_y - y

    target_distance = math.sqrt(dx**2 + dy**2)
    if target_distance < 0.05:
        current_waypoint += 1
        
        if current_waypoint >= len(waypoints):
            break
        
    
    distance_moved = (left_distance + right_distance) / 2
    
    target_angle = math.atan2(dy, dx)
    
    heading_error = target_angle - theta
    
    heading_error = math.atan2(
            math.sin(heading_error),
            math.cos(heading_error)
                )
                
    delta_theta = (right_distance - left_distance) / wheel_distance

    theta_mid = theta + delta_theta / 2

    x += distance_moved * math.cos(theta_mid)
    y += distance_moved * math.sin(theta_mid)

    theta += delta_theta

    # Keep theta between -pi and pi
    theta = math.atan2(math.sin(theta), math.cos(theta))


    # L-I-D-A-R

    
    scan = lidar.getRangeImage()
    layer = scan[:resolution]
    
    left_scan = layer[:170]
    front_scan = layer[170:342]
    right_scan = layer[342:]
    
    # To Find The Closest Obstacle In Each Region
    def closest_distance(scan):
        valid = [
            value
            for value in scan
            if math.isfinite(value)
            ]
        if valid:
            return min(valid)
        return float("inf")
        
    left_closest_dist = closest_distance(left_scan)
    right_closest_dist = closest_distance(right_scan)
    front_closest_dist = closest_distance(front_scan)    
    
    valid_points = [
        (i, distance)
        for i, distance in enumerate(layer)
        if math.isfinite(distance)
        ]
        
    if valid_points:
        closest_index, closest_distance = min(
            valid_points, key = lambda x: x[1]
            )
            
        obstacle_angle = -fov / 2 + closest_distance * fov / (resolution - 1)
    else:
         obstacle_angle = 0
         closest_distance = float("inf")
            
    if front_closest_dist < obstacle_threshold:
        if left_closest_dist > right_closest_dist:
            left_speed = -avoidance_speed
            right_speed = avoidance_speed
            
        else:
            left_speed = avoidance_speed
            right_speed = -avoidance_speed
    
    else:  
        

        # Correcting Heading Errors

        heading_correction = K_heading * heading_error
        
        left_speed = base_speed - heading_correction
        right_speed = base_speed + heading_correction
        
        
    left_speed = max(-6.28, min(6.28, left_speed))
    right_speed = max(-6.28, min(6.28, right_speed))
    
    
    print(
        f"GPS: ({gps_x:.4f}, {gps_y:.4f}, {gps_z:.4f}) | "
        f"Distance To Target: {target_distance:.3f} | "
        f"Current Target: ({target_x}, {target_y}) | "
        f"ODOM: "
        f"({x:.3f}, {y:.3f}) "
        f"θ={math.degrees(theta):.1f}° | "
        f"GPS LOCAL: ({gps_local_x:.3f}, {gps_local_y:.3f}) | "
        f"ERROR IN OD-to-GPS: "
        f"({error_x:.3f}, {error_y:.3f}) | "
        f"HEADING ERROR: {heading_error:.3f} | "
        f"OBSTACLE: {closest_distance:.3f}, {math.degrees(obstacle_angle):.3f}"
    )
        #  Move


    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)