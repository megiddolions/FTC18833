package org.firstinspires.ftc.teamcode.vison.pipelines.align;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Map;
import java.util.TreeMap;

import static org.firstinspires.ftc.teamcode.lib.CvUtil.filterContours;
import static org.firstinspires.ftc.teamcode.lib.CvUtil.findContours;

import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.camera_width;
import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.camera_height;

@Config
public class BluePowerShootsAlignPipeLine extends AlignPipeLine {
    private final static Rect view_rect = new Rect(100, 30, 900, 120);

    public static double filterContoursMinArea = 40;
    public static double filterContoursMinPerimeter = 0;
    public static double filterContoursMinWidth = 10;
    public static double filterContoursMaxWidth = 30;
    public static double filterContoursMinHeight = 0;
    public static double filterContoursMaxHeight = 1000;
    public static double[] filterContoursSolidity = {0, 100};
    public static double filterContoursMaxVertices = 1000000;
    public static double filterContoursMinVertices = 0;
    public static double filterContoursMinRatio = 0;
    public static double filterContoursMaxRatio = 1000;

    private final ArrayList<MatOfPoint> findContoursOutput = new ArrayList<>();
    private final ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<>();

    private final Rect[] objects = new Rect[3];

    public VisionTarget.PowerShoot target = VisionTarget.PowerShoot.Right;

    @Override
    public double getError() {
//        RobotUniversal.telemetry.addData("target", target);
        if (objects[0] == null)
            return 0;
        return camera_width/2d - (objects[target.offset].x + objects[target.offset].width/2d);
    }

    @Override
    public double getDistance() {
        return 0;
    }

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
            Rect object_rect = Imgproc.boundingRect(object);
            object_rect.x += view_rect.x;
            object_rect.y += view_rect.y;
            Imgproc.rectangle(input, object_rect, new Scalar(0, 150, 0), 4);
        }

        cal_position(input);

        Imgproc.rectangle(input, view_rect, new Scalar(0, 255, 0), 1);

        draw_vertical_line(input, camera_width/2, new Scalar(255, 255, 255), 2);
        draw_vertical_line(input, (int)(camera_width/2-getError()), new Scalar(200, 100, 100), 2);

        Bitmap map = Bitmap.createBitmap(camera_width, camera_height, Bitmap.Config.RGB_565);
        Utils.matToBitmap(input, map);

        FtcDashboard.getInstance().sendImage(map);

        return input;
    }

    private void cal_position(Mat frame) {
        Map<Integer, Rect> map = new TreeMap<>();

        for (MatOfPoint object : filterContoursOutput) {
            Rect object_rect = Imgproc.boundingRect(object);

            map.put(object_rect.x, object_rect);
        }

        Rect[] sorted_objects = new Rect[map.values().size()];
        map.values().toArray(sorted_objects);

        if (sorted_objects.length >= 3) {
            objects[0] = sorted_objects[0];
            objects[1] = sorted_objects[1];
            objects[2] = sorted_objects[2];

            for (int i = 0; i < 3; i++) {
                objects[i].x += view_rect.x;
                objects[i].y += view_rect.y;
                Imgproc.rectangle(frame, objects[i], i == target.offset ? new Scalar(255, 175, 0) : new Scalar(255, 255, 0), 4);
            }
        } else {
            objects[0] = null;
        }
    }
}
