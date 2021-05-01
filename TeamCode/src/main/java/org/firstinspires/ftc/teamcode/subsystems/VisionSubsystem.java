package org.firstinspires.ftc.teamcode.subsystems;

import org.commandftc.Subsystem;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.Constants.VisionConstants;
import org.firstinspires.ftc.teamcode.vison.pipelines.CountRingsPipeLine;
import org.firstinspires.ftc.teamcode.vison.pipelines.RingPipeLine;
import org.firstinspires.ftc.teamcode.vison.pipelines.SamplePipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.commandftc.RobotUniversal.*;

public class VisionSubsystem extends Subsystem {
//    public final CameraStreamServer server = CameraStreamServer.getInstance();
    public final OpenCvCamera camera;
    private final  RingPipeLine ringProcess;
    private final SamplePipeline pipeLine;
    public int rings;

    public VisionSubsystem() {
        ringProcess = new RingPipeLine();
        pipeLine = new SamplePipeline();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.setPipeline(pipeLine);
//        telemetry.addData("Shapes", () -> countRingsPipeLine.nonzero);

        camera.openCameraDeviceAsync(() -> camera.startStreaming(VisionConstants.camera_width, VisionConstants.camera_height, OpenCvCameraRotation.UPRIGHT));

//        server.setSource(camera);
//        server.onOpModePreStart(opMode);
        telemetry.addData("orange", () -> rings);
    }

    public int count_rings() {
        ringProcess.processFrame(pipeLine.last_input);
        int orange_pixels = ringProcess.orange_pixels;
        if (orange_pixels < 1000) {
            return 0;
        } else if (orange_pixels < 3500) {
            return 1;
        } else {
            return 4;
        }
    }
}
