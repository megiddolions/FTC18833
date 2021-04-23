package org.firstinspires.ftc.teamcode.vison;

import android.sax.StartElementListener;

import org.firstinspires.ftc.teamcode.Robot;
import org.opencv.core.Mat;
import org.opencv.videoio.VideoWriter;
import org.openftc.easyopencv.OpenCvPipeline;

public class SaveVideoPipeLine extends OpenCvPipeline {

    @Override
    public Mat processFrame(Mat input) {
        Robot.opMode.telemetry.addData("frame size", input.size());
        return input;
    }
}
