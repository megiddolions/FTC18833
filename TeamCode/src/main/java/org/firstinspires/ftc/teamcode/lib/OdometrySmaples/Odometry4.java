package org.firstinspires.ftc.teamcode.lib.OdometrySmaples;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;

import static org.firstinspires.ftc.teamcode.Constants.DriveTrainConstants.kOdometryConstants;

public class Odometry4 implements OdometryInterface {
    private double prevLeftEncoder = 0, prevRightEncoder = 0, prevHorizontalEncoder = 0;
    private Rotation2d previousAngle = new Rotation2d();
    Pose2d robotPose = new Pose2d();

    @Override
    public double getX() {
        return robotPose.getX();
    }

    @Override
    public double getY() {
        return robotPose.getY();
    }

    @Override
    public double getA() {
        return robotPose.getRotation().getRadians();
    }

    @Override
    public void update(double left, double right, double center) {
        double deltaLeftEncoder = (left - prevLeftEncoder) * kOdometryConstants.meters_per_tick;
        double deltaRightEncoder = (right - prevRightEncoder) * kOdometryConstants.meters_per_tick;
        double deltaHorizontalEncoder = (center - prevHorizontalEncoder) * kOdometryConstants.meters_per_tick;

        Rotation2d angle = previousAngle.plus(
                new Rotation2d(
                        (deltaLeftEncoder - deltaRightEncoder) / kOdometryConstants.getVerticalWheelsDistance()
                )
        );

        prevLeftEncoder = left;
        prevRightEncoder = right;
        prevHorizontalEncoder = center;

        double dw = (angle.minus(previousAngle).getRadians());

        double dx = (deltaLeftEncoder + deltaRightEncoder) / 2;
        double dy = deltaHorizontalEncoder - (kOdometryConstants.getHorizontalWheelOffset() * dw);

        Twist2d twist2d = new Twist2d(dx, dy, dw);

        Pose2d newPose = robotPose.exp(twist2d);

        previousAngle = angle;

        robotPose = new Pose2d(newPose.getTranslation(), angle);
    }
}
