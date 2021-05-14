package org.firstinspires.ftc.teamcode.lib.kinematics;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface MecanumDrive extends TankDrive, Subsystem {
    void driveLeft(double power);
    void driveRight(double power);

    void driveDirection(double x, double y, double a, double power);
}
