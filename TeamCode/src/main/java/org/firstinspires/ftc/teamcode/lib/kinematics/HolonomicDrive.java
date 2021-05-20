package org.firstinspires.ftc.teamcode.lib.kinematics;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface HolonomicDrive extends Subsystem {
    void driveDirection(double x, double y, double a, double power);
}
