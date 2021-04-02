package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.vison.RingPipeLine;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.PipelineRecordingParameters;

@Disabled
@TeleOp(name="OpenCV Test")
public class OpenCVTest extends LinearOpMode  {
    VisionSubsystem vision;

    @Override
    public void runOpMode() {
        vision = new VisionSubsystem(hardwareMap);
        waitForStart();

        while (opModeIsActive())
            ;
    }
}