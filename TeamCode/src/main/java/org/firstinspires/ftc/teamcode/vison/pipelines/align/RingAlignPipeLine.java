package org.firstinspires.ftc.teamcode.vison.pipelines.align;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.camera_height;
import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.camera_width;
import static org.firstinspires.ftc.teamcode.lib.CvUtil.*;

public class RingAlignPipeLine extends AlignPipeLine {
    public final static double filterContoursMinArea = 1000;
    public final static double filterContoursMinPerimeter = 0;
    public final static double filterContoursMinWidth = 0;
    public final static double filterContoursMaxWidth = 1000;
    public final static double filterContoursMinHeight = 0;
    public final static double filterContoursMaxHeight = 10000;
    public final static double[] filterContoursSolidity = {0, 100};
    public final static double filterContoursMaxVertices = 1000000;
    public final static double filterContoursMinVertices = 0;
    public final static double filterContoursMinRatio = 0;
    public final static double filterContoursMaxRatio = 1000;

    private final ArrayList<MatOfPoint> findContoursOutput = new ArrayList<>();
    private final ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<>();

    private Rect target = new Rect();

    @Override
    public double getError() {
        return camera_width/2d - target.x - target.width/2d;
    }

    @Override
    public double getDistance() {
        return 0;
    }

    @Override
    public Mat processFrame(Mat input) {

        Mat hsv_out = new Mat();
        filter_orange(input, hsv_out);
        // Find and filter contours
        findContours(hsv_out, false, findContoursOutput);
        filterContours(findContoursOutput, filterContoursMinArea, filterContoursMinPerimeter,
                filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight,
                filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices,
                filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);

        hsv_out.release();

        Rect max_object = new Rect();
        double max_area = 0;

        for (MatOfPoint object : filterContoursOutput) {
            Rect object_rect = Imgproc.boundingRect(object);
            Imgproc.rectangle(input, object_rect, new Scalar(0, 255, 0), 2);
            if (object_rect.area() >= max_area){
                max_area = object_rect.area();
                max_object = object_rect;
            }
        }

        target = max_object;

        Imgproc.rectangle(input, target, new Scalar(255, 0, 0), 3);

        draw_vertical_line(input, target.x + target.width/2, new Scalar(255, 0, 0), 2);
        draw_horizontal_line(input, target.y + target.height/2, new Scalar(255, 0, 0), 2);
        draw_vertical_line(input, camera_width/2, new Scalar(255, 255, 255), 2);

        Bitmap map = Bitmap.createBitmap(camera_width, camera_height, Bitmap.Config.RGB_565);
        Utils.matToBitmap(input, map);

        FtcDashboard.getInstance().sendImage(map);

        return input;
    }
}
