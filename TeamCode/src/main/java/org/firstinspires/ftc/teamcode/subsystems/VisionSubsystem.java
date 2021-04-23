package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.Constants.VisionConstants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.SubsystemBase;
import org.firstinspires.ftc.teamcode.vison.RingPipeLine;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class VisionSubsystem extends SubsystemBase {
    public CameraStreamServer server = CameraStreamServer.getInstance();
    public OpenCvCamera camera;

    public VisionSubsystem() {
        int camera_id = Robot.opMode.hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId",
                        "id",
                        Robot.opMode.hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                Robot.opMode.hardwareMap.get(WebcamName.class, "Webcam 1"), camera_id);

        camera.setPipeline(new RingPipeLine());

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(VisionConstants.camera_width, VisionConstants.camera_height, OpenCvCameraRotation.UPRIGHT);
            }
        });

        server.setSource(camera);
//        server.onOpModePreInit(Robot.opMode);
        server.onOpModePreStart(Robot.opMode);
    }
}
