package org.firstinspires.ftc.teamcode.lib.OdometrySmaples;

import org.firstinspires.ftc.teamcode.Constants;

public class Odometry1 implements OdometryInterface {
    private final LocalCoordinateSystem lcs = new LocalCoordinateSystem();

    @Override
    public double getX() {
        return lcs.x;
    }

    @Override
    public double getY() {
        return lcs.y;
    }

    @Override
    public double getA() {
        return lcs.a;
    }

    @Override
    public void update(double left, double right, double center) {
        lcs.update(
                left,
                right,
                center
        );
    }

    private static class LocalCoordinateSystem {
        public double x = 0;    // The approximated x position of the robot relative to where it started
        public double y = 0;    // The approximated y position of the robot relative to where it started
        public double a = 0;    // The approximated heading of the robot relative to its initial heading

        public double prev_le;
        public double prev_re;
        public double prev_ce;


        private final double ENCODER_CPR          = Constants.DriveTrainConstants.kOdometryConstants.ticks_per_revolution;             // Counts per full rotation of an encoder
        private final double ROBOT_DIAMETER       = Constants.DriveTrainConstants.kOdometryConstants.getVerticalWheelsDistance();      //15.7075609922;    //15.74735 //15.53           // Distance between the left and right encoder (diameter) in inches
        private final double CENTER_WHEEL_OFFSET  = Constants.DriveTrainConstants.kOdometryConstants.getHorizontalWheelOffset();      //7.725136416;      //7.719 //7.375 Distance of the center encoder to the line made between the left and right encoders (radius) in inches

        private final double WHEEL_DIAMETER     = Constants.DriveTrainConstants.kOdometryConstants.wheel_diameter;

        private final double METER_PER_COUNT   = WHEEL_DIAMETER * Math.PI / ENCODER_CPR;

        public void update(double le, double re, double ce) {

            // Calculate encoder deltas
            double ld = le - prev_le;
            double rd = re - prev_re;
            double cd = ce - prev_ce;

            // Calculate phi, or the delta of our angle
            double ph = (rd * METER_PER_COUNT - ld * METER_PER_COUNT) / ROBOT_DIAMETER;

            // The arclength of movement forward/backward
            double dc = (rd * METER_PER_COUNT + ld * METER_PER_COUNT) / 2;

            // The arclength of movement left/right
            double sc = (cd * METER_PER_COUNT) + (ph * CENTER_WHEEL_OFFSET);

            // Calculate the new angle of the robot using the difference between the left and right encoder
            a = (re * METER_PER_COUNT - le * METER_PER_COUNT) / ROBOT_DIAMETER;

            // Calculate the new position of the robot by adding the arc vector to the absolute pos
            double sinph = Math.sin(ph);
            double cosph = Math.cos(ph);

            double s;
            double c;

            // If the arc turn is small enough, do this instead to avoid a div by zero error
            if(Math.abs(ph) < 1E-9) {
                s = 1.0 - 1.0 / 6.0 * ph * ph;
                c = 0.5 * ph;
            } else {
                s = sinph / ph;
                c = (1.0 - cosph) / ph;
            }

            // Find our x and y translations relative to the origin pose (0,0,0)
            double rel_x = sc * s - dc * c;
            double rel_y = sc * c - dc * s;

            // Transform those x and y translations to the actual rotation of our robot, and translate our robots positions to the new spot
            x -= rel_x * Math.cos(a) - rel_y * Math.sin(a);
            y -= rel_x * Math.sin(a) + rel_y * Math.cos(a);

            // Used to calculate deltas for next loop
            prev_le = le;
            prev_re = re;
            prev_ce = ce;

        }
    }
}
