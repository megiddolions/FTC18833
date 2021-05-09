package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants.VisionConstants;
import org.firstinspires.ftc.teamcode.vison.pipelines.align.AlignPipeLine;
import org.firstinspires.ftc.teamcode.vison.pipelines.RingPipeLine;
import org.firstinspires.ftc.teamcode.vison.pipelines.align.BlueWobellAlignPipeLine;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.commandftc.RobotUniversal.hardwareMap;

public class VisionSubsystem extends SubsystemBase {
    public final OpenCvCamera camera;
    private final AlignPipeLine align_pipeLine;
    private final RingPipeLine ringPipeLine;

    public VisionSubsystem() {
        align_pipeLine = new BlueWobellAlignPipeLine();
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
        } else if (orange_pixels < 2500) {
            return 1;
        } else {
            return 4;
        }
    }

    public double getError() {
        return align_pipeLine.getError();
    }

    public int getOrangePixels() {
        return ringPipeLine.orange_pixels;
    }
}
