from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

lidar = robot.getDevice("lidar")
lidar.enable(timestep)

while robot.step(timestep) != -1:

    point_cloud = lidar.getRangeImage()

    print(point_cloud)