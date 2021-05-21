package org.firstinspires.ftc.teamcode.lib.OdometrySmaples;

import org.firstinspires.ftc.teamcode.Constants;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class Odometry6 implements OdometryInterface {
    Pose2d wheel_1 = new Pose2d(Constants.DriveTrainConstants.kOdometryConstants.leftWheel, new Rotation2d());
    Pose2d wheel_2 = new Pose2d(Constants.DriveTrainConstants.kOdometryConstants.rightWheel, new Rotation2d());
    Pose2d wheel_3 = new Pose2d(Constants.DriveTrainConstants.kOdometryConstants.horizontalWheel, Rotation2d.fromDegrees(90));

    @Override
    public double getX() {
        return 0;
    }

    @Override
    public double getY() {
        return 0;
    }

    @Override
    public double getA() {
        return 0;
    }

    @Override
    public void update(double left, double right, double center) {

    }
}
