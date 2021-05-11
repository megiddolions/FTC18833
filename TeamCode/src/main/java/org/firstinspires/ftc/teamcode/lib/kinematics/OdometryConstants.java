package org.firstinspires.ftc.teamcode.lib.kinematics;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class OdometryConstants {
    public final Translation2d leftWheel;
    public final Translation2d rightWheel;
    public final Translation2d horizontalWheel;

    public OdometryConstants(Translation2d leftWheel, Translation2d rightWheel, Translation2d horizontalWheel) {
        this.leftWheel = leftWheel;
        this.rightWheel = rightWheel;
        this.horizontalWheel = horizontalWheel;
    }

    public double getHorizontalWheelsDistance() {
        return leftWheel.getDistance(rightWheel);
    }
}
