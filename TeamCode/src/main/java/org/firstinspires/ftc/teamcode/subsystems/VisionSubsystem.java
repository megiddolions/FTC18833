package org.firstinspires.ftc.teamcode.subsystems;

import org.commandftc.Subsystem;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.Constants.VisionConstants;
import org.firstinspires.ftc.teamcode.vison.RingPipeLine;
import org.firstinspires.ftc.teamcode.vison.SaveVideoPipeLine;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.commandftc.RobotUniversal.*;

public class VisionSubsystem extends Subsystem {
    public CameraStreamServer server = CameraStreamServer.getInstance();
    public OpenCvCamera camera;

    public VisionSubsystem() {

        int camera_id = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), camera_id);

        camera.setPipeline(new RingPipeLine());

        camera.openCameraDeviceAsync(() -> camera.startStreaming(VisionConstants.camera_width, VisionConstants.camera_height, OpenCvCameraRotation.UPRIGHT));

        server.setSource(camera);
        server.onOpModePreStart(opMode);
        telemetry.addData("opMode", () -> opMode);

    }
}
