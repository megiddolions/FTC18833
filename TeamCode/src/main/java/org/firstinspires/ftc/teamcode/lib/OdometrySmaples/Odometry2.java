package org.firstinspires.ftc.teamcode.lib.OdometrySmaples;

import static org.firstinspires.ftc.teamcode.Constants.DriveTrainConstants.kOdometryConstants;

public class Odometry2 implements OdometryInterface {
    private double previousLeft = 0;
    private double previousRight = 0;
    private double previousCenter = 0;

    private double x = 0;
    private double y = 0;
    private double heading = 0;

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
        return heading;
    }

    @Override
    public void update(double left, double right, double center) {
        double deltaLeft =  left - previousLeft;
        double deltaRight = right - previousRight;
        double deltaCenter = center - previousCenter;

        previousLeft = left;
        previousRight = right;
        previousCenter = center;

        double deltaHeading = (deltaRight - deltaLeft)/(kOdometryConstants.meters_per_tick);
        heading = normalizeRadians((right - left)/(kOdometryConstants.meters_per_tick));

//        deltaHorizontal = deltabEncoder - (deltaHeading*BENCODER_OFFSET); //takes away the bEncoder counts that were a result of turning
//
//        deltay = (deltarEncoder + deltalEncoder)/2; //robot centric y and x
//        deltax = deltaHorizontal;

        double deltax;
        double deltay;

        if(deltaHeading == 0){
            deltax = deltaCenter;
            deltay = (deltaLeft + deltaRight)/2;
        }else{
            double turnRadius = kOdometryConstants.meters_per_tick*(deltaLeft + deltaRight)/(deltaRight - deltaLeft);
            double strafeRadius = deltaCenter/deltaHeading - kOdometryConstants.meters_per_tick;

            deltax = turnRadius*(Math.cos(deltaHeading) - 1) + strafeRadius*Math.sin(deltaHeading);
            deltay = turnRadius*Math.sin(deltaHeading) + strafeRadius*(1 - Math.cos(deltaHeading));
        }

        x += deltax;
        x += deltay;
    }

    public static double normalizeRadians(double angle){
        while(angle >= 2*Math.PI) {
            angle -= 2*Math.PI;
        }
        while(angle < 0.0) {
            angle += 2*Math.PI;
        }
        return angle;
    }
}
