package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.commandftc.RobotUniversal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vison.pipelines.TowerPipeLine;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name="Text Vision")
public class TestVision extends LinearOpMode {
    private OpenCvCamera camera;
    TowerPipeLine pipeLine;

    @Override
    public void runOpMode() {
        RobotUniversal.telemetry = telemetry;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeLine = new TowerPipeLine();
        camera.setPipeline(pipeLine);

        telemetry.addData("error", this::getError);
        telemetry.update();

        camera.openCameraDeviceAsync(() -> camera.startStreaming(Constants.VisionConstants.camera_width, Constants.VisionConstants.camera_height, OpenCvCameraRotation.UPRIGHT));



        while(opModeIsActive())
            telemetry.update();

        waitForStart();
    }

    public double getError() {
        return (Constants.VisionConstants.camera_width) / 2.0 - (pipeLine.right - pipeLine.left) / 2.0;
    }
}