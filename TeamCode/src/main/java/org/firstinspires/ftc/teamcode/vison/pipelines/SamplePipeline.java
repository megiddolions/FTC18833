package org.firstinspires.ftc.teamcode.vison.pipelines;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class SamplePipeline extends OpenCvPipeline {
    public Mat last_input;

    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */

    @Override
    public Mat processFrame(Mat input) {
        last_input = input;
        /*
         * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
         * will only dereference to the same image for the duration of this particular
         * invocation of this method. That is, if for some reason you'd like to save a copy
         * of this particular frame for later use, you will need to either clone it or copy
         * it to another Mat.
         */

        /*
         * Draw a simple box around the middle 1/2 of the entire frame
         */
//        Imgproc.rectangle(
//                input,
//                new Point(0, 300),
//                new Point(200, 500),
//                new Scalar(0, 255, 0), 4);

        /**
         * NOTE: to see how to get data from your pipeline to your OpMode as well as how
         * to change which stage of the pipeline is rendered to the viewport when it is
         * tapped, please see {@link PipelineStageSwitchingExample}
         */

        return input;
    }

//    @Override
//    public void onViewportTapped() {
//        /*
//         * The viewport (if one was specified in the constructor) can also be dynamically "paused"
//         * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
//         * when you need your vision pipeline running, but do not require a live preview on the
//         * robot controller screen. For instance, this could be useful if you wish to see the live
//         * camera preview as you are initializing your robot, but you no longer require the live
//         * preview after you have finished your initialization process; pausing the viewport does
//         * not stop running your pipeline.
//         *
//         * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
//         */
//
//        viewportPaused = !viewportPaused;
//
//        if (viewportPaused) {
//            webcam.pauseViewport();
//        } else {
//            webcam.resumeViewport();
//        }
//    }
}