package org.firstinspires.ftc.teamcode.vison.pipelines.align;

import android.util.Pair;
import android.util.Range;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.vison.VisionTarget;
import org.jetbrains.annotations.PropertyKey;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.TreeSet;

import static org.firstinspires.ftc.teamcode.lib.CvUtil.*;
import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.*;

public class BluePowerShootsAlignPipeLine extends AlignPipeLine {
    public final static double[] blue_hsvThresholdHue = {75, 140};
    public final static double[] blue_hsvThresholdSaturation = {127, 255};
    public final static double[] blue_hsvThresholdValue = {100, 255};

    public final static double filterContoursMinArea = 100;
    public final static double filterContoursMinPerimeter = 0;
    public final static double filterContoursMinWidth = 0;
    public final static double filterContoursMaxWidth = 1000;
    public final static double filterContoursMinHeight = 25;
    public final static double filterContoursMaxHeight = 50;
    public final static double[] filterContoursSolidity = {0, 100};
    public final static double filterContoursMaxVertices = 1000000;
    public final static double filterContoursMinVertices = 0;
    public final static double filterContoursMinRatio = 0;
    public final static double filterContoursMaxRatio = 1000;

    private final ArrayList<MatOfPoint> findContoursOutput = new ArrayList<>();
    private final ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<>();


    private final static Rect view_rect = new Rect(0, 0, camera_width, 55);

    private Rect[] targets = {new Rect(), new Rect(), new Rect()};

    public enum PowerShoot {
        left(0),
        center(1),
        right(2);

        final int index;
        PowerShoot(int index) {
            this.index = index;
        }
    }
    private PowerShoot target = PowerShoot.left;
    public void setPowerShoot(PowerShoot target) {
        this.target = target;
    }

    @Override
    public double getError() {
        Rect target_rect = targets[this.target.index];
        return camera_width / 2.0 - (target_rect.x + target_rect.width / 2.0);
    }

    public double getError(PowerShoot target) {
        Rect target_rect = targets[target.index];
        return camera_width / 2.0 - (target_rect.x + target_rect.width / 2.0);
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat view = input.submat(view_rect);
        Mat hsv_out = new Mat();
        hsvThreshold(view, blue_hsvThresholdHue, blue_hsvThresholdSaturation, blue_hsvThresholdValue, hsv_out);
        view.release();

        findContours(hsv_out, false, findContoursOutput);
        hsv_out.release();
        filterContours(findContoursOutput, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);

        for (MatOfPoint object : filterContoursOutput) {
            Rect object_rect = Imgproc.boundingRect(object);
            Imgproc.rectangle(input, object_rect, new Scalar(255, 0, 0), 2);
        }

        for (Rect target : targets) {
            Imgproc.rectangle(input, target, new Scalar(255, 255, 0), 3);
        }

        calc_positions();

        Imgproc.line(input, new Point(camera_width / 2.0, 0),
                new Point(camera_width / 2.0, camera_height),
                new Scalar(255, 255, 255),
                6
        );

        double target_x = targets[target.index].x + targets[target.index].width/2.0;
        Imgproc.line(input, new Point(target_x, 0),
                new Point(target_x, camera_height),
                new Scalar(255, 175, 0),
                6
        );

        Imgproc.rectangle(input, view_rect, new Scalar(0, 255, 0), 2);
        return input;
    }

    private void calc_positions() {
        Rect[] targets = {new Rect(), new Rect(), new Rect()};

        TreeMap<Integer, Rect> values_map = new TreeMap<>();
        for (MatOfPoint object : filterContoursOutput) {
            Rect object_rect = Imgproc.boundingRect(object);
            values_map.put(object_rect.x, object_rect);
        }
        Rect[] values = new Rect[3];
        int i = 0;
        for (Map.Entry<Integer, Rect> p: values_map.entrySet()) {
            values[i++] = p.getValue();
            if (i == 3)
                break;
        }

        System.arraycopy(values, 0, targets, 0, Math.min(values_map.size(), 3));
        this.targets = targets;
    }
}
