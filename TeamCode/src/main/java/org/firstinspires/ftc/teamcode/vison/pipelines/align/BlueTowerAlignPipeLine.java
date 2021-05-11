package org.firstinspires.ftc.teamcode.vison.pipelines.align;

import org.commandftc.RobotUniversal;
import org.firstinspires.ftc.teamcode.Constants;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.lib.CvUtil.filterContours;
import static org.firstinspires.ftc.teamcode.lib.CvUtil.findContours;
import static org.firstinspires.ftc.teamcode.lib.CvUtil.hsvThreshold;

// TODO: Add more filters for vision

public class BlueTowerAlignPipeLine extends AlignPipeLine {
    public final static double[] blue_hsvThresholdHue = {75, 140};
    public final static double[] blue_hsvThresholdSaturation = {127, 255};
    public final static double[] blue_hsvThresholdValue = {100, 255};

    public final static double filterContoursMinArea = 400;
    public final static double filterContoursMinPerimeter = 0;
    public final static double filterContoursMinWidth = 0;
    public final static double filterContoursMaxWidth = 1000;
    public final static double filterContoursMinHeight = 0;
    public final static double filterContoursMaxHeight = 1000;
    public final static double[] filterContoursSolidity = {0, 100};
    public final static double filterContoursMaxVertices = 1000000;
    public final static double filterContoursMinVertices = 0;
    public final static double filterContoursMinRatio = 0;
    public final static double filterContoursMaxRatio = 1000;

    private final ArrayList<MatOfPoint> findContoursOutput = new ArrayList<>();
    private final ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<>();

    public final static Rect view_rect = new Rect(0, 0, Constants.VisionConstants.camera_width, 200);

    private Rect target1 = new Rect();
    private Rect target2 = new Rect();

    @Override
    public double getError() {
        if (target1.equals(new Rect()) || target2.equals(new Rect()))
            return 0;
        return Constants.VisionConstants.camera_width / 2.0 - (target1.x + target2.x) / 2.0 + (target1.width + target2.width) / 4.0;
    }

    @Override
    public Mat processFrame(Mat input) {

        Mat view = input.submat(view_rect);
        Mat hsv_out = new Mat();
        hsvThreshold(view, blue_hsvThresholdHue, blue_hsvThresholdSaturation, blue_hsvThresholdValue, hsv_out);
        view.release();

        findContours(hsv_out, false, findContoursOutput);
        filterContours(findContoursOutput, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);
        hsv_out.release();

        try {
            if (filterContoursOutput.size() >= 2)
                calc_positions();
        } catch (IndexOutOfBoundsException e) {
            target1 = new Rect();
            target2 = new Rect();
        }
        for (MatOfPoint object : filterContoursOutput) {
            Rect object_rect = Imgproc.boundingRect(object);
            Imgproc.rectangle(input, object_rect, new Scalar(255, 0, 0), 2);
        }
        Imgproc.rectangle(input, target1, new Scalar(255, 0, 255), 2);
        Imgproc.rectangle(input, target2, new Scalar(255, 100, 255), 2);

        Imgproc.line(input, new Point(Constants.VisionConstants.camera_width / 2.0, 0),
                new Point(Constants.VisionConstants.camera_width / 2.0, Constants.VisionConstants.camera_height),
                new Scalar(255, 255, 255),
                6
        );

        double middle = (target1.x + target2.x) / 2.0 + (target1.width + target2.width) / 4.0;
        Imgproc.line(input, new Point(middle, 0),
                new Point(middle, Constants.VisionConstants.camera_height),
                new Scalar(75, 75, 75),
                6
        );

        Imgproc.rectangle(input, view_rect, new Scalar(0, 255, 0), 4);
        return input;
    }

    private void calc_positions() {
        int index = 0;

        Rect target1 = null;
        Rect target2 = null;
        // No wobell goals or one or both targets are hidden by wobell goals
        if (filterContoursOutput.size() == 2) {
            this.target1 = Imgproc.boundingRect(filterContoursOutput.get(0));
            this.target2 = Imgproc.boundingRect(filterContoursOutput.get(1));
            return;
        }
        // Get the real target if visable
        while (index < filterContoursOutput.size()) {
            Rect object_rect = Imgproc.boundingRect(filterContoursOutput.get(index++));
            if (object_rect.y + object_rect.height < 190) {
                target1 = object_rect;
                break;
            }
        }

        if (target1 != null) {
            filterContoursOutput.remove(index-1);
        } else {
            return;
        }
        index = 0;
        // Get second real target if visible
        while (index < filterContoursOutput.size()) {
            Rect object_rect = Imgproc.boundingRect(filterContoursOutput.get(index++));
            if (object_rect.y + object_rect.height < 190) {
                target2 = object_rect;
                break;
            }
        }

        if (target2 != null) {
            this.target1 = target1;
            this.target2 = target2;
            return;
        }

        index = 0;
        // Guess second target by width
        while (index < filterContoursOutput.size()) {
            Rect object_rect = Imgproc.boundingRect(filterContoursOutput.get(index++));
            if (target2 == null || object_rect.width > target2.width) {
                target2 = object_rect;
            }
        }

        if (target2 != null) {
            this.target1 = target1;
            this.target2 = target2;
            return;
        }

        this.target1 = new Rect();
        this.target2 = new Rect();
    }
}
