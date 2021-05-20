package org.firstinspires.ftc.teamcode.lib.OdometrySmaples;

import static org.firstinspires.ftc.teamcode.Constants.DriveTrainConstants.kOdometryConstants;

public class Odometry3 implements OdometryInterface {
    double x = 0;
    double y = 0;
    double theta = 0;

    private double previousLeft = 0;
    private double previousRight = 0;
    private double previousCenter = 0;

    @Override
    public double getX() {
        return x;
    }

    @Override
    public double getY() {
        return y;
    }

    @Override
    public double getA() {
        return theta;
    }

    @Override
    public void update(double left, double right, double center) {
        double deltaLeftDistance = (left - previousLeft) * kOdometryConstants.meters_per_tick;
        double deltaRightDistance = (right - previousRight) * kOdometryConstants.meters_per_tick;
        double deltaCenterDistance = (center - previousCenter) * kOdometryConstants.meters_per_tick;
        x  += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.cos(theta);
        y  += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.sin(theta);
        theta  += (deltaLeftDistance - deltaRightDistance) / kOdometryConstants.getVerticalWheelsDistance();

        previousLeft = left;
        previousRight = right;
        previousCenter = center;
    }
}
