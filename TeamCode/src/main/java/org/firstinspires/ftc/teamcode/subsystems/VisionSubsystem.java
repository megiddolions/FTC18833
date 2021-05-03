package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.commandftc.Subsystem;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants.VisionConstants;
import org.firstinspires.ftc.teamcode.Drive;
import org.firstinspires.ftc.teamcode.vison.pipelines.DrivePipeLine;
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
    public final DrivePipeLine align_pipeLine;
    private final RingPipeLine ringPipeLine;

    public VisionSubsystem() {
        align_pipeLine = new DrivePipeLine();
        ringPipeLine = new RingPipeLine();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.openCameraDeviceAsync(() -> camera.startStreaming(VisionConstants.camera_width, VisionConstants.camera_height, OpenCvCameraRotation.UPRIGHT));

    }

    public void set_for_drive() {
        camera.setPipeline(align_pipeLine);
    }

    public void set_for_autonomous() {
        camera.setPipeline(ringPipeLine);
    }

    public int count_rings() {
        int orange_pixels = ringPipeLine.orange_pixels;
        if (orange_pixels < 700) {
            return 0;
        } else if (orange_pixels < 3000) {
            return 1;
        } else {
            return 4;
        }
    }

    public double getError() {
        return (VisionConstants.camera_width) / 2.0 - (align_pipeLine.left() + align_pipeLine.right()) / 2.0;
    }

    public int getOrangePixels() {
        return ringPipeLine.orange_pixels;
    }
}
