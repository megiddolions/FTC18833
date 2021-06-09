package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Constants.VisionConstants;
import org.firstinspires.ftc.teamcode.vison.pipelines.align.BlueWobbleAlignPipeLine;
import org.firstinspires.ftc.teamcode.vison.pipelines.align.RingAlignPipeLine;
import org.firstinspires.ftc.teamcode.vison.pipelines.align.VisionTarget;
import org.firstinspires.ftc.teamcode.vison.pipelines.align.AlignPipeLine;
import org.firstinspires.ftc.teamcode.vison.pipelines.RingPipeLine;
import org.firstinspires.ftc.teamcode.vison.pipelines.align.BlueTowerAlignPipeLine;
import org.firstinspires.ftc.teamcode.vison.pipelines.align.NonePipeLine;
import org.firstinspires.ftc.teamcode.vison.pipelines.align.RedTowerAlignPipeLine;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.commandftc.RobotUniversal.hardwareMap;
import static org.commandftc.RobotUniversal.telemetry;

public class VisionSubsystem extends SubsystemBase {
    public final OpenCvCamera rearCamera;
    public final OpenCvCamera frontCamera;
    private final Map<VisionTarget, AlignPipeLine> align_pipeLines;
    private VisionTarget target = VisionTarget.None;
    private RingPipeLine ringPipeLine;
    private AlignPipeLine currentAlignPipeLine;
    public AlignPipeLine front_pipeline = new NonePipeLine();

    public VisionSubsystem() {
        frontCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "FrontWebcam"));
        rearCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "RearWebcam"));

        rearCamera.openCameraDeviceAsync(() -> rearCamera.startStreaming(VisionConstants.camera_width, VisionConstants.camera_height, OpenCvCameraRotation.UPRIGHT));
        frontCamera.openCameraDeviceAsync(() -> frontCamera.startStreaming(VisionConstants.camera_width, VisionConstants.camera_height, OpenCvCameraRotation.UPRIGHT));

        align_pipeLines = new HashMap<>();
        align_pipeLines.put(VisionTarget.BlueTower, new BlueTowerAlignPipeLine());
        align_pipeLines.put(VisionTarget.RedTower, new RedTowerAlignPipeLine());
//        align_pipeLines.put(VisionTarget.RingStack, new RingAlignPipeLine());
        align_pipeLines.put(VisionTarget.BlueWobble, new BlueWobbleAlignPipeLine());
//        align_pipeLines.put(VisionTarget.RedWobble, new RedWobbleAlignPipeLine());
//        align_pipeLines.put(VisionTarget.BluePowerShoots, new BluePowerShootsAlignPipeLine());
        align_pipeLines.put(VisionTarget.None, new NonePipeLine());
    }

    public void update_align_pipeline() {
        setAlignPipeLine(align_pipeLines.get(target));
    }

    public void setTarget(VisionTarget target) {
        this.target  = target;
        update_align_pipeline();
    }

    public void setAlignPipeLine(AlignPipeLine pipeLine) {
        currentAlignPipeLine = pipeLine;
        rearCamera.setPipeline(pipeLine);
    }

    public VisionTarget getTarget() {
        return target;
    }

    public void set_for_autonomous(Alliance alliance) {
        ringPipeLine = new RingPipeLine(alliance);
        rearCamera.setPipeline(ringPipeLine);
        telemetry.addData("pixels", () -> ringPipeLine.orange_pixels);
    }

    public int count_rings() {
        int orange_pixels = ringPipeLine.orange_pixels;
        if (orange_pixels < 400) {
            return 0;
        } else if (orange_pixels < 2500) {
            return 1;
        } else {
            return 4;
        }
    }

    public double getError() {
        if (currentAlignPipeLine == null)
            return 0;
        return Objects.requireNonNull(currentAlignPipeLine).getError();
    }

    public double getFrontError() {
        return front_pipeline.getError();
    }

    public int getOrangePixels() {
        return ringPipeLine.orange_pixels;
    }
}
