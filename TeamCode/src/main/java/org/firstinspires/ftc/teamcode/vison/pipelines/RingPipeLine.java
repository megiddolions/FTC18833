package org.firstinspires.ftc.teamcode.vison.pipelines;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.vison.pipelines.align.AlignPipeLine;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.camera_height;
import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.camera_width;

public class RingPipeLine extends OpenCvPipeline {

    public final static double[] hsvThresholdHue = {8, 52};
    public final static double[] hsvThresholdSaturation = {110.07193916564364, 255.0};
    public final static double[] hsvThresholdValue = {171, 255.0};

//    public Mat cvErodeKernel = new Mat();
    public int orange_pixels = 0;

    private final Rect ring_area_rect;

    public RingPipeLine(Alliance alliance) {
        switch (alliance) {
            case Blue:
                ring_area_rect = new Rect(0, 550, 50, 130);
                break;
            default:
            case Red:
                ring_area_rect = new Rect(1230, 550, 50, 130);
                break;
        }
    }
    
    @Override
    public Mat processFrame(Mat input) {
        Mat hsvThresholdOutput = new Mat();

        hsvThreshold(input, hsvThresholdOutput);

        Mat ring_area = hsvThresholdOutput.submat(ring_area_rect);

        orange_pixels = Core.countNonZero(ring_area);

        Imgproc.rectangle(
                input,
                ring_area_rect,
                new Scalar(0, 255, 0), 4);

        hsvThresholdOutput.release();
        ring_area.release();

        Bitmap map = Bitmap.createBitmap(camera_width, camera_height, Bitmap.Config.RGB_565);
        Utils.matToBitmap(input, map);

        FtcDashboard.getInstance().sendImage(map);

        return input;
    }

    /**
     * Segment an image based on hue, saturation, and value ranges.
     * @param input The image on which to perform the HSL threshold.
     * @param out The image in which to store the output.
     */
    private void hsvThreshold(Mat input,
                              Mat out) {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_RGB2HSV);
        Core.inRange(out, new Scalar(RingPipeLine.hsvThresholdHue[0], RingPipeLine.hsvThresholdSaturation[0], RingPipeLine.hsvThresholdValue[0]),
                new Scalar(RingPipeLine.hsvThresholdHue[1], RingPipeLine.hsvThresholdSaturation[1], RingPipeLine.hsvThresholdValue[1]), out);
    }

    /**
     * Filter out an area of an image using a binary mask.
     * @param input The image on which the mask filters.
     * @param mask The binary image that is used to filter.
     * @param output The image in which to store the output.
     */
    private void mask(Mat input, Mat mask, Mat output) {
        mask.convertTo(mask, CvType.CV_8UC1);
        Core.bitwise_xor(output, output, output);
        input.copyTo(output, mask);
    }
}
