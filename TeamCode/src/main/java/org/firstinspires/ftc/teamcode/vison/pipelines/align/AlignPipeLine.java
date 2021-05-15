package org.firstinspires.ftc.teamcode.vison.pipelines.align;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.teamcode.lib.CvUtil.cvBitwiseOr;
import static org.firstinspires.ftc.teamcode.lib.CvUtil.hsvThreshold;

public abstract class AlignPipeLine extends OpenCvPipeline {
    // BLUE filter //
    public final static double[] blue_hsvThresholdHue = {75, 140};
    public final static double[] blue_hsvThresholdSaturation = {127, 255};
    public final static double[] blue_hsvThresholdValue = {100, 255};
    // RED filter //
    public static final double[] red_hsvThresholdHueLow = {0.0, 12.081904427590233};
    public static final double[] red_hsvThresholdHueHigh = {156.47481988659865, 180.0};
    public static final double[] red_hsvThresholdSaturation = {102.4280604698675, 255.0};
    public static final double[] red_hsvThresholdValue = {4.586330935251798, 183.92490360924};

    public abstract double getError();

    public abstract double getDistance();

    protected static void filter_red(Mat src, Mat dst) {
        Mat hsvThresholdLowOutput = new Mat();
        hsvThreshold(src, red_hsvThresholdHueLow, red_hsvThresholdSaturation, red_hsvThresholdValue,
                hsvThresholdLowOutput);

        Mat hsvThresholdHighOutput = new Mat();
        hsvThreshold(src, red_hsvThresholdHueHigh, red_hsvThresholdSaturation, red_hsvThresholdValue,
                hsvThresholdHighOutput);

        cvBitwiseOr(hsvThresholdLowOutput, hsvThresholdHighOutput, dst);
        hsvThresholdLowOutput.release();
        hsvThresholdHighOutput.release();
    }

    protected static void filter_blue(Mat src, Mat dst) {
        hsvThreshold(src, blue_hsvThresholdHue, blue_hsvThresholdSaturation, blue_hsvThresholdValue, dst);

    }
}
