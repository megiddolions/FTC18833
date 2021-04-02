package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.SubsystemBase;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class VisionSubsystem extends SubsystemBase {
    public CameraStreamServer server = CameraStreamServer.getInstance();
    public OpenCvCamera camera;

    public VisionSubsystem(HardwareMap hardwareMap) {
        int camera_id = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), camera_id);

//        camera.setPipeline(new RingPipeLine());

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        server.setSource(camera);
//        server.onOpModePreInit(Robot.getInstance().opMode);
        server.onOpModePreStart(Robot.getInstance().opMode);
    }
}
