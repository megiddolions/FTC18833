package org.firstinspires.ftc.teamcode.vison;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.lib.Util;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.commandftc.RobotUniversal.*;

public class SaveVideoPipeLine extends OpenCvPipeline {

    @SuppressLint("SdCardPath")
    @Override
    public Mat processFrame(Mat input) {
        telemetry.addData("frame size", input.size());
        if (opMode.gamepad1.a) {
            Mat out = new Mat();
            Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2RGB);
            String filepath = "/sdcard/images/" + Util.getTime() + ".png";
            Imgcodecs.imwrite(filepath, out);
            telemetry.addData("filepath", filepath);
        }
        return input;
    }
}
