package org.firstinspires.ftc.teamcode.vison.pipelines;

import org.firstinspires.ftc.teamcode.Constants;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import static org.commandftc.RobotUniversal.telemetry;

public class DrivePipeLine extends OpenCvPipeline {
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

    public DrivePipeLine() {
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
        hsvThreshold(input, hsvThresholdOutput);

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
                new Point(Constants.VisionConstants.camera_width/2,0),
                new Point(Constants.VisionConstants.camera_width/2, Constants.VisionConstants.camera_height),
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

    /**
     * Segment an image based on hue, saturation, and value ranges.
     * @param input The image on which to perform the HSL threshold.
     * @param out The image in which to store the output.
     */
    private void hsvThreshold(Mat input,
                              Mat out) {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_RGB2HSV);
        Core.inRange(out, new Scalar(blue_hsvThresholdHue[0], blue_hsvThresholdSaturation[0], blue_hsvThresholdValue[0]),
                new Scalar(blue_hsvThresholdHue[1], blue_hsvThresholdSaturation[1], blue_hsvThresholdValue[1]), out);
    }

    /**
     * Filter out an area of an image using a binary mask.
     * @param input The image on which the mask filters.
     * @param mask The binary image that is used to filter.
     * @param output The image in which to store the output.
     */
    private void mask(Mat input, Mat mask, Mat output) {
        mask.convertTo(mask, CvType.CV_8UC1);
        Core.bitwise_xor(output, output, output);
        input.copyTo(output, mask);
    }

    /**
     * Sets the values of pixels in a binary image to their distance to the nearest black pixel.
     * @param input The image on which to perform the Distance Transform.
     */
    private void findContours(Mat input, boolean externalOnly,
                              List<MatOfPoint> contours) {
        Mat hierarchy = new Mat();
        contours.clear();
        int mode;
        if (externalOnly) {
            mode = Imgproc.RETR_EXTERNAL;
        }
        else {
            mode = Imgproc.RETR_LIST;
        }
        int method = Imgproc.CHAIN_APPROX_SIMPLE;
        Imgproc.findContours(input, contours, hierarchy, mode, method);
    }


    /**
     * Filters out contours that do not meet certain criteria.
     * @param inputContours is the input list of contours
     * @param output is the the output list of contours
     * @param minArea is the minimum area of a contour that will be kept
     * @param minPerimeter is the minimum perimeter of a contour that will be kept
     * @param minWidth minimum width of a contour
     * @param maxWidth maximum width
     * @param minHeight minimum height
     * @param maxHeight maximimum height
     * @param minVertexCount minimum vertex Count of the contours
     * @param maxVertexCount maximum vertex Count
     * @param minRatio minimum ratio of width to height
     * @param maxRatio maximum ratio of width to height
     */
    private void filterContours(List<MatOfPoint> inputContours, double minArea,
                                double minPerimeter, double minWidth, double maxWidth, double minHeight, double
                                        maxHeight, double[] solidity, double maxVertexCount, double minVertexCount, double
                                        minRatio, double maxRatio, List<MatOfPoint> output) {
        final MatOfInt hull = new MatOfInt();
        output.clear();
        //operation
        for (int i = 0; i < inputContours.size(); i++) {
            final MatOfPoint contour = inputContours.get(i);
            final Rect bb = Imgproc.boundingRect(contour);
            if (bb.width < minWidth || bb.width > maxWidth) continue;
            if (bb.height < minHeight || bb.height > maxHeight) continue;
            final double area = Imgproc.contourArea(contour);
            if (area < minArea) continue;
            if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter) continue;
            Imgproc.convexHull(contour, hull);
            MatOfPoint mopHull = new MatOfPoint();
            mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
            for (int j = 0; j < hull.size().height; j++) {
                int index = (int)hull.get(j, 0)[0];
                double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1]};
                mopHull.put(j, 0, point);
            }
            final double solid = 100 * area / Imgproc.contourArea(mopHull);
            if (solid < solidity[0] || solid > solidity[1]) continue;
            if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)	continue;
            final double ratio = bb.width / (double)bb.height;
            if (ratio < minRatio || ratio > maxRatio) continue;
            output.add(contour);
        }
    }
}
