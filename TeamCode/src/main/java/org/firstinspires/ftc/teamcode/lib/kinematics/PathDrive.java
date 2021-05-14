package org.firstinspires.ftc.teamcode.lib.kinematics;

import edu.wpi.first.wpilibj.geometry.Pose2d;

public interface PathDrive {
    void moveTo(Pose2d position, double speed);
}
