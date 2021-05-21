package org.firstinspires.ftc.teamcode.lib.OdometrySmaples;

import org.firstinspires.ftc.teamcode.Constants;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

public class Odometry5 implements OdometryInterface {
    double prev_left_encoder_pos = 0;
    double prev_right_encoder_pos = 0;
    double prev_center_encoder_pos = 0;

    double x=0, y=0, heading=0;
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
        double delta_left_encoder_pos = left - prev_left_encoder_pos;
        double delta_right_encoder_pos = right - prev_right_encoder_pos;
        double delta_center_encoder_pos = center - prev_center_encoder_pos;

        double phi = (delta_left_encoder_pos - delta_right_encoder_pos) / Constants.DriveTrainConstants.kOdometryConstants.getHorizontalWheelOffset();
        double delta_middle_pos = (delta_left_encoder_pos + delta_right_encoder_pos) / 2;
        double delta_perp_pos = delta_center_encoder_pos - (Constants.DriveTrainConstants.kOdometryConstants.getHorizontalWheelOffset()) * phi;

        double delta_x = delta_middle_pos * cos(heading) - delta_perp_pos * sin(heading);
        double delta_y = delta_middle_pos * sin(heading) + delta_perp_pos * cos(heading);

        x += delta_x;
        y += delta_y;
        heading += phi;

        prev_left_encoder_pos = left;
        prev_right_encoder_pos = right;
        prev_center_encoder_pos = center;

    }
}
