from controller import Robot, Motor, DistanceSensor
robot = Robot()
timestep = int(robot.getBasicTimeStep())
target = 90
base_speed = 3.0
turn_speed = 2.0
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

initial_left = left_encoder.getValue()
initial_right = right_encoder.getValue()

#  motor = robot.getDevice('motorname')
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

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
    values = [sensor.getValue() for sensor in sensors]
    
    front = (values[0] + values[7]) / 2
    right = (values[1] + values[2]) / 2
    left = (values[5] + values[6]) / 2
    
    error = right - target
    P = Kp * error
    
    integral_error += error
    integral_error = max(-1000, min(1000, integral_error))
    I = Ki * integral_error
    
    dt = 1000
    derivative = (error - previous_error) / dt
    D = Kd * derivative
    
    correction = P + I + D
    
    previous_error = error
    
    if front > caution_threshold:
        caution_speed = 1.5
        left_speed = caution_speed - correction
        right_speed = caution_speed + correction
    
    elif front > danger_threshold:
        if left < right:
            left_speed = -2.0
            right_speed = 2.0
        else:
            left_speed = 2.0
            right_speed = -2.0
    
    left_speed = base_speed - correction
    right_speed = base_speed + correction
    
    left_speed =  max(-6.28, min(6.28, left_speed))
    right_speed = max(-6.28, min(6.28, right_speed))
    
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    
    print(
        f"Error: {error:.3f} | "
        f"Correction : {correction:.3f}"
        )
    