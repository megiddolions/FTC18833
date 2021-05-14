package org.firstinspires.ftc.teamcode.lib.kinematics;

public interface MecanumDrive extends TankDrive {
    void driveLeft(double power);
    void driveRight(double power);

    void driveDirection(double x, double y, double a, double power);
}
