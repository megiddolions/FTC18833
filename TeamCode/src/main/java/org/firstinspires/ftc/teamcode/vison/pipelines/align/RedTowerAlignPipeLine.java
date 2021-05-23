package org.firstinspires.ftc.teamcode.vison.pipelines.align;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Map;
import java.util.TreeMap;

import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.camera_width;
import static org.firstinspires.ftc.teamcode.lib.CvUtil.filterContours;
import static org.firstinspires.ftc.teamcode.lib.CvUtil.findContours;

public class RedTowerAlignPipeLine extends AlignPipeLine {
    private final static Rect view_rect = new Rect(0, 0, camera_width, 400);

    public final static double filterContoursMinArea = 600;
    public final static double filterContoursMinPerimeter = 0;
    public final static double filterContoursMinWidth = 0;
    public final static double filterContoursMaxWidth = 35;
    public final static double filterContoursMinHeight = 0;
    public final static double filterContoursMaxHeight = 200;
    public final static double[] filterContoursSolidity = {0, 100};
    public final static double filterContoursMaxVertices = 1000000;
    public final static double filterContoursMinVertices = 0;
    public final static double filterContoursMinRatio = 0;
    public final static double filterContoursMaxRatio = 1000;

    private final ArrayList<MatOfPoint> findContoursOutput = new ArrayList<>();
    private final ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<>();

    private Rect left;
    private Rect right;

    @Override
    public Mat processFrame(Mat input) {

        Mat view = input.submat(view_rect);
        // Filter for red
        Mat hsv_out = new Mat();
        filter_blue(view, hsv_out);
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
            Imgproc.rectangle(input, Imgproc.boundingRect(object), new Scalar(0, 150, 0), 4);
        }
        cal_position(input);
        // Show view area on stream
        Imgproc.rectangle(input, view_rect, new Scalar(0, 255, 0), 2);

        draw_vertical_line(input, camera_width/2, new Scalar(255, 255, 255), 2);
        draw_vertical_line(input, (int)(camera_width/2-getError()), new Scalar(200, 100, 100), 2);
        return input;
    }
    @Override
    public double getError() {
        if (left == null || right == null)
            return 0;
        return camera_width/2d - (left.x + left.width/2d + right.x + right.width/2d)/2d;
    }

    @Override
    public double getDistance() {
        return 0;
    }

    private void cal_position(Mat frame) {
        Map<Integer, Rect> map = new TreeMap<>();

        for (MatOfPoint object : filterContoursOutput) {
            Rect object_rect = Imgproc.boundingRect(object);

            map.put(-object_rect.x, object_rect);
        }

        Rect[] sorted_objects = new Rect[map.values().size()];
        map.values().toArray(sorted_objects);

        if (sorted_objects.length >= 3) {
            left = sorted_objects[0];
            right = new Rect(
                    Math.min(sorted_objects[1].x, sorted_objects[2].x),
                    Math.min(sorted_objects[1].y, sorted_objects[2].y),
                    Math.max(sorted_objects[1].width, sorted_objects[2].width),
                    sorted_objects[1].height + sorted_objects[2].height);

            Imgproc.rectangle(frame, left, new Scalar(255, 255, 0), 4);
            Imgproc.rectangle(frame, right, new Scalar(255, 150, 0), 4);
        } else if (sorted_objects.length == 2) {
            left = sorted_objects[0];
            right = sorted_objects[1];
        } else {
            left = null;
            right = null;
        }
    }
}
