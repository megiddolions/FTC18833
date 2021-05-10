package org.firstinspires.ftc.teamcode.vison.pipelines.align;

import org.firstinspires.ftc.teamcode.Constants;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.lib.CvUtil.*;

public class BlueWobellAlignPipeLine extends AlignPipeLine {
    public final static double[] blue_hsvThresholdHue = {75, 140};
    public final static double[] blue_hsvThresholdSaturation = {127, 255};
    public final static double[] blue_hsvThresholdValue = {0.0, 255};

    private final ArrayList<MatOfPoint> findContoursOutput = new ArrayList<>();

    private Rect target = new Rect();

    public final static Rect view_rect = new Rect(0, 200,
            Constants.VisionConstants.camera_width, 50);

    @Override
    public double getError() {
        return Constants.VisionConstants.camera_width / 2.0 - target.x - target.width / 2.0;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.rectangle(input, view_rect, new Scalar(0, 175, 50), 4);

        Mat view = input.submat(view_rect);
        Mat hsv_out = new Mat();
        hsvThreshold(view, blue_hsvThresholdHue, blue_hsvThresholdSaturation, blue_hsvThresholdValue, hsv_out);
        view.release();

        findContours(hsv_out, false, findContoursOutput);

        target = new Rect();

        for (MatOfPoint object : findContoursOutput) {
            Rect object_rect = Imgproc.boundingRect(object);
            object_rect.y += view_rect.y;
            if (object_rect.height > target.height) {
                target = object_rect;
            } else {
                Imgproc.rectangle(input, object_rect, new Scalar(255, 0, 0), 2);
            }
        }

        view.release();

        Imgproc.rectangle(input, target, new Scalar(255, 0, 255), 4);

        Imgproc.line(input, new Point(Constants.VisionConstants.camera_width / 2.0, 0),
                new Point(Constants.VisionConstants.camera_width / 2.0, Constants.VisionConstants.camera_height),
                new Scalar(255, 255, 255),
                6
        );

        Imgproc.line(input, new Point(target.x + target.width / 2.0, 0),
                new Point(target.x + target.width / 2.0, Constants.VisionConstants.camera_height),
                new Scalar(75, 75, 75),
                6
        );

        Imgproc.putText(input, "error: " + getError(),
                new Point(30, 150),
                1,
                3,
                new Scalar(175,0,20),
                3);
        return input;
    }
}
