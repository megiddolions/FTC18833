package org.firstinspires.ftc.teamcode.vison.pipelines.align;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.teamcode.Constants;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.camera_height;
import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.camera_width;
import static org.firstinspires.ftc.teamcode.lib.CvUtil.filterContours;
import static org.firstinspires.ftc.teamcode.lib.CvUtil.findContours;
import static org.firstinspires.ftc.teamcode.lib.CvUtil.mask;

public class BlueWobbleAlignPipeLine extends AlignPipeLine {
    public final static Rect view_rect = new Rect(0, 480,
            Constants.VisionConstants.camera_width, 50);

    private final ArrayList<MatOfPoint> findContoursOutput = new ArrayList<>();

    Rect target = new Rect();

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
        Mat view = input.submat(view_rect);
        // Filter for red
        Mat hsv_out = new Mat();
        filter_blue(view, hsv_out);
        view.release();
        // Find and filter contours
        findContours(hsv_out, false, findContoursOutput);
        hsv_out.release();
        // Draw contours
        target = new Rect();

        for (MatOfPoint object : findContoursOutput) {
            Rect object_rect = Imgproc.boundingRect(object);
            object_rect.y += view_rect.y;
            object_rect.x += view_rect.x;
            if (object_rect.height > target.height) {
                target = object_rect;
            } else {
                Imgproc.rectangle(input, object_rect, new Scalar(255, 0, 0), 2);
            }
        }

        // Show view area on stream
        Imgproc.rectangle(input, view_rect, new Scalar(0, 255, 0), 2);


        Imgproc.rectangle(input, target, new Scalar(255, 255, 0), 2);

        draw_vertical_line(input, camera_width/2, new Scalar(255, 255, 255), 2);
        draw_vertical_line(input, (int)(camera_width/2-getError()), new Scalar(200, 100, 100), 2);

        Bitmap map = Bitmap.createBitmap(camera_width, camera_height, Bitmap.Config.RGB_565);
        Utils.matToBitmap(input, map);

        FtcDashboard.getInstance().sendImage(map);

        return input;
    }
}
