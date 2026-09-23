from controller import Robot
import math

robot = Robot()
timestep = int(robot.getBasicTimeStep())

lidar = robot.getDevice("lidar")
lidar.enable(timestep)

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

#print("Horizontal resolution:", lidar.getHorizontalResolution())
#print("Number of layers:", lidar.getNumberOfLayers())
#print("Field of view:", lidar.getFov())
#print("Vertical field of view:", lidar.getVerticalFov())
#print("Minimum range:", lidar.getMinRange())
#print("Maximum range:", lidar.getMaxRange())

fov = lidar.getFov()
resolution = lidar.getHorizontalResolution()

print("Resolution: ", resolution)
print("FOV: ", fov)

while robot.step(timestep) != -1:
    scan = lidar.getRangeImage()
    layer = scan[:resolution]
    
    left_motor.setVelocity(-2.0)
    right_motor.setVelocity(2.0)
    
    valid_points = [
        (i, distance)
        for i, distance in enumerate(layer)
        if math.isfinite(distance)
                    ]
    if valid_points:
        closest_index, closest_distance = min(
            valid_points,
            key = lambda x: x[1]
            )
        angle = -fov / 2 + (closest_index * fov / (resolution - 1))
        
        #print(
        #    f"Index: {i:3d} | "
        #    f"Angle: {math.degrees(angle):7.2f} | "
        #    f"Distance: {distance}"
        #    )
        print(
            f"Closest obstacle: "
            f"{closest_distance:.3f} m | "
            f"Index: {closest_index} | "
            f"Angle: {math.degrees(angle):.2f}°"
            )
    """
    while robot.step(timestep) != -1:
        left_motor.setVelocity(2.0)
        right_motor.setVelocity(-2.0)
        point_cloud = lidar.getRangeImage()
    
        #print(point_cloud)
        print("Number of readings:", len(point_cloud))
        #print("First 10:", point_cloud[:10])
        for i in [1120, 1440, 1536, 1568, 1600, 1632, 195]:
            print(i, point_cloud[i])
    
    """