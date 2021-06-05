package org.firstinspires.ftc.teamcode.tests;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.commandftc.opModes.CommandBasedTeleOp;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.DriveSideWaysCommand;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.HorizontalAlignCommand;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.TankDriveCommand;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.TurnCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.vison.pipelines.ViewPipeLine;
import org.firstinspires.ftc.teamcode.vison.pipelines.align.RingAlignPipeLine;

@TeleOp(name="FrontCameraTest", group = "tests")
public class FrontCameraTest extends CommandBasedTeleOp {
    protected DriveTrainSubsystem driveTrain;
    protected VisionSubsystem vision;
    @Override
    public void assign() {
        vision = new VisionSubsystem();
        driveTrain = new DriveTrainSubsystem();
        vision.front_pipeline = new RingAlignPipeLine();
        vision.frontCamera.setPipeline(vision.front_pipeline);

        driveTrain.setDefaultCommand(new TankDriveCommand(driveTrain, () -> -gamepad1.left_stick_y, () -> -gamepad1.right_stick_y));
        gp1.left_stick_button().whenHeld(
                new DriveForwardCommand(driveTrain, () -> -gamepad1.right_stick_y * 0.5));
        gp1.right_stick_button().whenHeld(
                new DriveForwardCommand(driveTrain, () -> -gamepad1.left_stick_y * 0.5));
        gp1.left_bumper().whileHeld(new DriveSideWaysCommand(driveTrain, () -> 0.5));
        gp1.right_bumper().whileHeld(new DriveSideWaysCommand(driveTrain, () -> -0.5));

        gp1.y().whenHeld(new HorizontalAlignCommand(driveTrain, vision));

        telemetry.addData("Error", vision::getFrontError);
        telemetry.addData("RearCameraVision(ms)", vision.rearCamera::getPipelineTimeMs);
        telemetry.addData("FrontCameraVision(ms)", vision.frontCamera::getPipelineTimeMs);
    }
}
