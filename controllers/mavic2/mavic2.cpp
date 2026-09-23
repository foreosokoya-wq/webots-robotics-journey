#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Gyro.hpp>
#include <vector>
#include <string>

using namespace webots;

int main(int argc, char **argv) {
    // Initialize the Robot instance
    Robot *robot = new Robot();
    int time_step = (int)robot->getBasicTimeStep();

    // 1. Initialize Sensors (IMU and Gyroscope)
    InertialUnit *imu = robot->getInertialUnit("inertial unit");
    imu->enable(time_step);

    Gyro *gyro = robot->getGyro("gyro");
    gyro->enable(time_step);

    // 2. Initialize the 4 Motors
    std::vector<std::string> motor_names = {
        "front_left_motor", "front_right_motor", 
        "rear_left_motor", "rear_right_motor"
    };
    std::vector<Motor*> motors;

    // Set motors to velocity control mode
    for (const std::string& name : motor_names) {
        Motor *motor = robot->getMotor(name);
        motor->setPosition(INFINITY);
        motor->setVelocity(0.0);
        motors.push_back(motor);
    }

    // Main simulation loop
    while (robot->step(time_step) != -1) {
        // Read sensor data
        const double *rpy = imu->getRollPitchYaw();
        const double *ang_vel = gyro->getValues();

        // Example: Apply a base velocity to lift off
        double base_speed = 10.0;
        for (Motor *motor : motors) {
            motor->setVelocity(base_speed);
        }
    }

    delete robot;
    return 0;
}
