package org.firstinspires.ftc.teamcode.vison.pipelines.align;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.camera_width;
import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.camera_height;
import static org.firstinspires.ftc.teamcode.lib.CvUtil.cvErode;
import static org.firstinspires.ftc.teamcode.lib.CvUtil.filterContours;
import static org.firstinspires.ftc.teamcode.lib.CvUtil.findContours;
import static org.firstinspires.ftc.teamcode.lib.CvUtil.mask;

public class BlueTowerAlignPipeLine extends AlignPipeLine {
    private final static Rect view_rect = new Rect(0, 0, camera_width, 400);

    public final static double filterContoursMinArea = 400;
    public final static double filterContoursMinPerimeter = 0;
    public final static double filterContoursMinWidth = 0;
    public final static double filterContoursMaxWidth = 0;
    public final static double filterContoursMinHeight = 200;
    public final static double filterContoursMaxHeight = 1000;
    public final static double[] filterContoursSolidity = {0, 100};
    public final static double filterContoursMaxVertices = 1000000;
    public final static double filterContoursMinVertices = 0;
    public final static double filterContoursMinRatio = 0;
    public final static double filterContoursMaxRatio = 1000;

    private final ArrayList<MatOfPoint> findContoursOutput = new ArrayList<>();
    private final ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<>();

    @Override
    public Mat processFrame(Mat input) {

        Mat view = input.submat(view_rect);
        // Filter for red
        Mat hsv_out = new Mat();
        filter_red(view, hsv_out);
        view.release();
        // Find and filter contours
        findContours(hsv_out, false, findContoursOutput);
        hsv_out.release();
        filterContours(findContoursOutput, filterContoursMinArea, filterContoursMinPerimeter,
                filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight,
                filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices,
                filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);
        // Draw contours
        for (MatOfPoint object : filterContoursOutput) {
            Imgproc.rectangle(input, Imgproc.boundingRect(object), new Scalar(255, 255, 0), 4);
        }
        // Show view area on stream
        Imgproc.rectangle(input, view_rect, new Scalar(0, 255, 0), 2);
        return input;
    }
    @Override
    public double getError() {
        return 0;
    }

    @Override
    public double getDistance() {
        return 0;
    }

}
