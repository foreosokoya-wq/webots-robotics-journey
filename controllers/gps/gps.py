from controller import Robot
import math
import random

gps_noise_std = 0.05
R = gps_noise_std ** 2        # 0.0025
Q = 1e-6                      # per-step; we'll tune this
kf_x = 0.0
P = 0.01
log = []

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# =========================
# GPS
# =========================

gps = robot.getDevice("gps")
gps.enable(timestep)


# =========================
# Wheel encoders
# =========================

left_encoder = robot.getDevice("left wheel sensor")
right_encoder = robot.getDevice("right wheel sensor")

left_encoder.enable(timestep)
right_encoder.enable(timestep)


# =========================
# Robot geometry
# =========================

wheel_radius = 0.0200#0
wheel_separation = 0.052


# =========================
# Initial sensor values
# =========================

robot.step(timestep)

initial_left = left_encoder.getValue()
initial_right = right_encoder.getValue()

previous_left = initial_left
previous_right = initial_right

initial_gps = gps.getValues()
gps_x0 = initial_gps[0]
gps_y0 = initial_gps[1]


# =========================
# Odometry pose
# =========================

x = 0.0
y = 0.0
theta = 0.0

waypoints = [
    (0, 1), (1, 1), (1, 0), (0,0)
]
current_waypoint = 0

# =========================
# Motors
# =========================

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))


# Move straight forward
left_motor.setVelocity(4.0)
right_motor.setVelocity(4.0)


# =========================
# Main loop
# =========================

while robot.step(timestep) != -1:

    # -------------------------
    # Encoder readings
    # -------------------------

    current_left = left_encoder.getValue()
    current_right = right_encoder.getValue()

    left_change = current_left - previous_left
    right_change = current_right - previous_right

    left_distance = left_change * wheel_radius
    right_distance = right_change * wheel_radius


    # -------------------------
    # Differential-drive odometry
    # -------------------------

    distance = (left_distance + right_distance) / 2

    delta_theta = (
        right_distance - left_distance
    ) / wheel_separation

    theta_mid = theta + delta_theta / 2

    x += distance * math.cos(theta_mid)
    y += distance * math.sin(theta_mid)

    theta += delta_theta

    theta = math.atan2(
        math.sin(theta),
        math.cos(theta)
    )
    


    # -------------------------
    # GPS
    # -------------------------

    gps_values = gps.getValues()

    gps_x = gps_values[0] - gps_x0
    gps_y = gps_values[1] - gps_y0
    
    gps_meas_x = random.gauss(0, gps_noise_std) + gps_x
    gps_meas_y = random.gauss(0, gps_noise_std) + gps_y

    
    # PREDICT
    dx = distance * math.cos(theta_mid)
    kf_x += dx
    P += Q
    
    # UPDATE
    K = P / (P + R)
    kf_x = kf_x + K * (gps_meas_x - kf_x)
    P = (1 - K) * P
    
    fixed_x = 0.3 * x + 0.7 * gps_meas_x
    log.append(
        (robot.getTime(),
         gps_x, x,
          gps_meas_x,
           fixed_x, kf_x,
            K, P))
        
    if gps_x >= 2.0:
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        break
    # -------------------------
    # Localization error
    # -------------------------

    #error_x = gps_x - localized_x
    #error_y = gps_y - localized_y

    # Apply GPS correction
    correction_gain = 0.8
    
    
    odom_weight = 0.3
    gps_weight = 0.7
     
    #position_error = math.sqrt(
    #    error_x**2 + error_y**2
    #)


    # -------------------------
    # Print
    # -------------------------
    """
    print(
        f"ODOM: ({x:.4f}, {y:.4f}) | "
        f"GPS: ({gps_x:.4f}, {gps_y:.4f}) | "
        f"ERROR: ({error_x:.4f}, {error_y:.4f}) | "
        f"CORRECTION: ({corrected_x:.4f}, {corrected_y:.4f}) | "
        f"MAG: {position_error:.4f} | "
        f"θ: {math.degrees(theta):.4f}°"
    )
    
    print(
        f"ODOM: ({x:.3f}, {y:.3f}) | "
        f"GPS: ({gps_x:.3f}, {gps_y:.3f}) | "
    )    

    # -------------------------
    # Update previous encoders
    # -------------------------
    """
    previous_left = current_left
    previous_right = current_right

import csv
with open("kf_run.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "gps_true", "odom", "gps_meas", "fixed", "kf", "K", "P"])
    w.writerows(log)    