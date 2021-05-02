package org.firstinspires.ftc.teamcode.subsystems;

import org.commandftc.Subsystem;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants.VisionConstants;
import org.firstinspires.ftc.teamcode.vison.pipelines.RingPipeLine;
import org.firstinspires.ftc.teamcode.vison.pipelines.SamplePipeline;
import org.firstinspires.ftc.teamcode.vison.pipelines.TowerPipeLine;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.commandftc.RobotUniversal.*;

public class VisionSubsystem extends Subsystem {
//    public final CameraStreamServer server = CameraStreamServer.getInstance();
    public final OpenCvCamera camera;
    private final  RingPipeLine ringProcess;
    private final SamplePipeline pipeLine;
    private final TowerPipeLine towerPipeLine;
    public int rings;

    public VisionSubsystem() {
        ringProcess = new RingPipeLine();
        pipeLine = new SamplePipeline();
        towerPipeLine = new TowerPipeLine();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.setPipeline(pipeLine);

        camera.openCameraDeviceAsync(() -> camera.startStreaming(VisionConstants.camera_width, VisionConstants.camera_height, OpenCvCameraRotation.UPRIGHT));

    }

    public int count_rings() {
        ringProcess.processFrame(pipeLine.last_input);
        int orange_pixels = ringProcess.orange_pixels;
        if (orange_pixels < 1000) {
            return 0;
        } else if (orange_pixels < 3000) {
            return 1;
        } else {
            return 4;
        }
    }

    public double getError() {
        towerPipeLine.processFrame(pipeLine.last_input);
        return (VisionConstants.camera_width) / 2.0 - (towerPipeLine.right - towerPipeLine.left) / 2.0;
    }
}
