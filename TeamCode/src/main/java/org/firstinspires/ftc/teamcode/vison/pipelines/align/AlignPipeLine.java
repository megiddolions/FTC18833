package org.firstinspires.ftc.teamcode.vison.pipelines.align;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.teamcode.lib.CvUtil.cvBitwiseOr;
import static org.firstinspires.ftc.teamcode.lib.CvUtil.hsvThreshold;

@Config
public abstract class AlignPipeLine extends OpenCvPipeline {
    // BLUE filter //
    public static double[] blue_hsvThresholdHue = {75, 140};
    public static double[] blue_hsvThresholdSaturation = {127, 255};
    public static double[] blue_hsvThresholdValue = {110, 255};
    // RED filter //
    public static double[] red_offset_hsvThresholdHue= {115, 145};
    public static double[] red_offset_hsvThresholdSaturation = {120, 255.0};
    public static double[] red_offset_hsvThresholdValue = {100, 255};

    public abstract double getError();

    public abstract double getDistance();

    protected static void filter_red(Mat src, Mat dst) {
        Mat hsv_color_offset90 = new Mat();
        Imgproc.cvtColor(src, hsv_color_offset90, Imgproc.COLOR_RGB2BGR);
        hsvThreshold(hsv_color_offset90, red_offset_hsvThresholdHue, red_offset_hsvThresholdSaturation, red_offset_hsvThresholdValue, dst);
        hsv_color_offset90.release();
    }

    protected static void filter_blue(Mat src, Mat dst) {
        hsvThreshold(src, blue_hsvThresholdHue, blue_hsvThresholdSaturation, blue_hsvThresholdValue, dst);
    }

    protected static void draw_vertical_line(Mat img, int x, Scalar color, int thickness) {
        Imgproc.line(img, new Point(x, 0), new Point(x, img.cols()-1), color, thickness);
    }
}
