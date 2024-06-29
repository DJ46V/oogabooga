#include "robot-config.h"
brain Brain;
// Make a controller and name it Greg
controller Greg = controller();
controller Beethoven = controller(partner);

// Front Left Wheel (FL)
motor FL = motor(PORT1, gearSetting::ratio18_1, true);
TestDriveMotor(FL);
// Front Right Wheel (FR)
motor FR = motor(PORT20, gearSetting::ratio18_1, false);
TestDriveMotor(FR);
// Back Left Wheel (BL)
motor BL = motor(PORT4, gearSetting::ratio18_1, true);
TestDriveMotor(BL);
// Back Right Wheel (BR)
motor BR = motor(PORT5, gearSetting::ratio18_1, false);
TestDriveMotor(BR);

MotorGroup leftWheels = MotorGroup(BL, FL);
MotorGroup rightWheels = MotorGroup(BR, FR);

/****************************
 *
 * The numbers there are the values the inertial sensor gives you for a full rotation in the positive and negative directions
 *      This can be found via the devices screen on the brain
 *
 ****************************/
Inertial angler = Inertial(PORT16, 358.0, 358.0);

// Positioner init

/**********************************
 *
 * Initialize tracking wheels, these examples use the rotation sensors, but you can use encoders or motors as well
 *
 **********************************/
Positioner::encoderArr arrX = {TrackingWheel(PORT14, true, 2.77)};
Positioner::encoderArr arrY = {TrackingWheel(PORT15, true, 2.77)};

// Odometry
Positioner positioner = Positioner(arrX, arrY, angler, {0, 0});

/***************************
 *
 * Initialize chassis here, this data must be accurate for the numbers to work
 *
 ***************************/
Chassis chassis = Chassis(leftWheels, rightWheels, positioner, 10.5, 36.0 / 60.0, 3.25 / 2.0, gearSetting::ratio18_1);

/**********************
 *
 * The constants here are defaulted and work for most cases, but you can change them to fit your needs
 *
 **********************/
PathFollowSettings purePursuitSettings = PathFollowSettings();
PurePursuitController purePursuit = PurePursuitController(
    PID(6.25, 0.1, 2.4325, 200, 6, 1),
    purePursuitSettings
        .setBrakeMode(PathFollowSettings::exitMode::normal)
        .setExitDist(2)
        .setUseDistToGoal(true)
        .setFollowPathDist(16)
        .setVirtualPursuitDist(11));

PathFollowSettings ramseteSettings = PathFollowSettings();
RamseteController ramsete = RamseteController(
    0.0108, 0.05,
    ramseteSettings
        .setBrakeMode(PathFollowSettings::exitMode::normal)
        .setExitDist(2)
        .setUseDistToGoal(true)
        .setFollowPathDist(12)
        .setVirtualPursuitDist(2));

PathFollowSettings pidSettings = PathFollowSettings();
PidController pidController = PidController(
    PID(9.25, 0.1, 2.4325, 200, 6, 1),
    PID(0.9, 0, 0.3, 0, 0, 0),
    pidSettings
        .setBrakeMode(PathFollowSettings::exitMode::normal)
        .setExitDist(1)
        .setUseDistToGoal(false)
        .setFollowPathDist(16)
        .setTurnAtStart(true)
        .setVirtualPursuitDist(9)
        .setTimeIn(200));

/************************
 *
 * These reversing functions are set for spinup, they might need to be changed next year
 *
 ************************/
PVector reverseAutonPosition(PVector v)
{
    return v * -1;
}
double reverseAutonAngle(double a)
{
    return posNeg180(a + 180);
}
WheelController wc = WheelController(
    &chassis,
    &ramsete, &purePursuit, &pidController,
    reverseAutonPosition, reverseAutonAngle,
    PID(1.5, 0, 4.6, -1, 20, 4),
    1.0);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void)
{
    leftWheels.setBrakeMode(hold);
    rightWheels.setBrakeMode(hold);
}
