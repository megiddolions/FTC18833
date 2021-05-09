package org.firstinspires.ftc.teamcode.vison.pipelines.align;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.vison.pipelines.align.AlignPipeLine;
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
import static org.firstinspires.ftc.teamcode.lib.CvUtil.mask;

public class BlueTowerAlignPipeLine extends AlignPipeLine {
    public final static double[] blue_hsvThresholdHue = {75, 140};
    public final static double[] blue_hsvThresholdSaturation = {127, 255};
    public final static double[] blue_hsvThresholdValue = {0.0, 255};

    public final static double filterContoursMinArea = 400.0;
    public final static double filterContoursMinPerimeter = 0.0;
    public final static double filterContoursMinWidth = 7.0;
    public final static double filterContoursMaxWidth = 40;
    public final static double filterContoursMinHeight = 100;
    public final static double filterContoursMaxHeight = 1000;
    public final static double[] filterContoursSolidity = {0, 100};
    public final static double filterContoursMaxVertices = 1000000;
    public final static double filterContoursMinVertices = 0;
    public final static double filterContoursMinRatio = 0.0;
    public final static double filterContoursMaxRatio = 1.0;

    private final ArrayList<MatOfPoint> findContoursOutput = new ArrayList<>();
    private final ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<>();

    public Rect left_rect;
    public Rect right_rect;

    public BlueTowerAlignPipeLine() {
//        telemetry.addData("Contours", filterContoursOutput::size);
    }

    public double left() {
        if (left_rect != null)
            return left_rect.x + left_rect.width / 2.0;
        else
            return 0;
    }

    public double right() {
        if (right_rect != null)
            return right_rect.x + right_rect.width / 2.0;
        else
            return 0;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat hsvThresholdOutput = new Mat();
        hsvThreshold(input, blue_hsvThresholdHue, blue_hsvThresholdSaturation, blue_hsvThresholdValue,
                hsvThresholdOutput);

        Mat output = new Mat();
        mask(input, hsvThresholdOutput, output);

        Mat ring_area = hsvThresholdOutput.submat(new Rect(
                new Point(0, 0),
                new Point(Constants.VisionConstants.camera_width, 200)
        ));

        // Step Find_Lines0:
        findContours(ring_area, false, findContoursOutput);
        filterContours(findContoursOutput, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);

        calc_positions(output);

        Imgproc.rectangle(
                output,
                new Point(0, 0),
                new Point(Constants.VisionConstants.camera_width, 200),
                new Scalar(0, 255, 0), 4);

        Imgproc.line(output,
                new Point(Constants.VisionConstants.camera_width/2.0,0),
                new Point(Constants.VisionConstants.camera_width/2.0, Constants.VisionConstants.camera_height),
                new Scalar(255, 255, 255), 4
        );

        Imgproc.line(output,
                new Point((left() + right())/2,0),
                new Point((left() + right())/2, Constants.VisionConstants.camera_height),
                new Scalar(255, 0, 0), 4
        );

        if (left_rect != null)
            Imgproc.rectangle(output, left_rect, new Scalar(255, 100, 255), 4);
        if (right_rect != null)
            Imgproc.rectangle(output, right_rect, new Scalar(255, 0, 255), 4);

        hsvThresholdOutput.release();
//        input.release();
        output.assignTo(input);
        output.release();
        ring_area.release();
        return input;
    }

    private void calc_positions(Mat frame) {
        Rect right_rect = null;
        Rect left_rect = null;
//        if (filterContoursOutput.size() == 2) {
//            left_rect = Imgproc.boundingRect(filterContoursOutput.get(0));
//            right_rect = Imgproc.boundingRect(filterContoursOutput.get(1));
//        } else {
            for (MatOfPoint object : filterContoursOutput) {
                Rect object_rect = Imgproc.boundingRect(object);
                Imgproc.rectangle(frame, object_rect, new Scalar(50, 150, 75), 4);
                if (object_rect.y + object_rect.height < 190) {
                    if (right_rect == null) {
                        right_rect = object_rect;
                    } else if (left_rect == null) {
                        left_rect = object_rect;
                    }
                }
//            }
        }

        if (left_rect != null) {
            this.left_rect = left_rect;
            this.right_rect = right_rect;
        }
    }

    @Override
    public double getError() {
        return (Constants.VisionConstants.camera_width) / 2.0 - (left() + right()) / 2.0;
    }
}
