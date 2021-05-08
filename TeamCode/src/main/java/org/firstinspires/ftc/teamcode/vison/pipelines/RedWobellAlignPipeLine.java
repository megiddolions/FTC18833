package org.firstinspires.ftc.teamcode.vison.pipelines;

import org.firstinspires.ftc.teamcode.Constants;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import static org.firstinspires.ftc.teamcode.lib.CvUtil.*;

import java.util.ArrayList;

public class RedWobellAlignPipeLine extends AlignPipeLine {
    public static final double[] hsvThresholdHueLow = {0.0, 12.081904427590233};
    public static final double[] hsvThresholdHueHigh = {156.47481988659865, 180.0};

    public static final double[] hsvThresholdSaturation = {102.4280604698675, 255.0};
    public static final double[] hsvThreshold0Value = {4.586330935251798, 183.92490360924};

    public static final double filterContoursMinArea = 300.0;
    public static final double filterContoursMinPerimeter = 0.0;
    public static final double filterContoursMinWidth = 7.0;
    public static final double filterContoursMaxWidth = 40;
    public static final double filterContoursMinHeight = 0;
    public static final double filterContoursMaxHeight = 1000;
    public static final double[] filterContoursSolidity = {0, 100};
    public static final double filterContoursMaxVertices = 1000000;
    public static final double filterContoursMinVertices = 0;
    public static final double filterContoursMinRatio = 0.0;
    public static final double filterContoursMaxRatio = 1.0;
    private final ArrayList<MatOfPoint> findContoursOutput = new ArrayList<>();
    private final ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<>();

    private Rect position = new Rect();

    @Override
    public Mat processFrame(Mat input) {
        Mat hsvThresholdLowOutput = new Mat();
        hsvThreshold(input, hsvThresholdHueLow, hsvThresholdSaturation, hsvThreshold0Value,
                hsvThresholdLowOutput);

        Mat hsvThresholdHighOutput = new Mat();
        hsvThreshold(input, hsvThresholdHueHigh, hsvThresholdSaturation, hsvThreshold0Value,
                hsvThresholdHighOutput);

        // combine hsv
        Mat hsvThresholdOutput = new Mat();
        cvBitwiseOr(hsvThresholdLowOutput, hsvThresholdHighOutput, hsvThresholdOutput);
        hsvThresholdLowOutput.release();
        hsvThresholdHighOutput.release();

        Mat output = new Mat();
        mask(input, hsvThresholdOutput, output);

        findContours(hsvThresholdOutput, false, findContoursOutput);
        filterContours(findContoursOutput, filterContoursMinArea, filterContoursMinPerimeter,
                filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight,
                filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices,
                filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio,
                filterContoursOutput);

        calc_positions(output);

        Imgproc.rectangle(
                output,
                new Point(0, 0),
                new Point(Constants.VisionConstants.camera_width, 200),
                new Scalar(0, 150, 0), 4);

        Imgproc.line(output,
                new Point(Constants.VisionConstants.camera_width/2.0,0),
                new Point(Constants.VisionConstants.camera_width/2.0, Constants.VisionConstants.camera_height),
                new Scalar(255, 255, 255), 4
        );

        Imgproc.line(output,
                new Point(position.x + (position.width)/2.0,0),
                new Point(position.x + (position.width)/2.0, Constants.VisionConstants.camera_height),
                new Scalar(40, 255, 40), 4
        );

        output.assignTo(input);
        output.release();

        return input;
    }

    private void calc_positions(Mat overlay) {
        for (MatOfPoint object : filterContoursOutput) {
            Rect object_rect = Imgproc.boundingRect(object);
            Imgproc.rectangle(overlay, object_rect, new Scalar(100, 100, 255), 4);
            position = object_rect;
        }
    }

    @Override
    public double getError() {
        return Constants.VisionConstants.camera_width / 2.0 - (position.x + position.width / 2.0);
    }
}
