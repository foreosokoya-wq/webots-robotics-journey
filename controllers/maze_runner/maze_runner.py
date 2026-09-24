from controller import Robot

robot = Robot()

timestep = int(robot.getBasicTimeStep())

WALL_THRESHOLD = 160

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

left_motor.setVelocity(2.0)
right_motor.setVelocity(2.0)

sensors = []

for i in range(8):
    sensor = robot.getDevice(f"ps{i}")
    sensor.enable(timestep)
    sensors.append(sensor)

def wall_detection(sensor_value):
    return sensor_value >= WALL_THRESHOLD

while robot.step(timestep) != -1:
    readings = [sensor.getValue() for sensor in sensors]
    front = readings[0] + readings[7]
    right = readings[1] + readings[2]
    back = readings[3] + readings[4]
    left = readings[5] + readings[6]
    direction = {
        "front": wall_detection(front),
        "right": wall_detection(right),
        "back": wall_detection(back),
        "left": wall_detection(left)
    }
    wall = {
        "front": wall_detection(max(readings[7], readings[0])),
        "right": wall_detection(max(readings[1], readings[2])),
        "back": wall_detection(max(readings[3], readings[4])),
        "left": wall_detection(max(readings[5], readings[6]))
    }
    print(direction)