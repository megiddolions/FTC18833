package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.commandftc.opModes.CommandBasedTeleOp;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.AlignRobotVisionCommand;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.TankDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.vison.pipelines.align.BluePowerShootsAlignPipeLine;

import edu.wpi.first.wpilibj2.command.InstantCommand;

@TeleOp(name="Vision Test")
public class VisionTest extends CommandBasedTeleOp {
    private DriveTrainSubsystem driveTrain;
    private VisionSubsystem vision;

    private BluePowerShootsAlignPipeLine powerShootsAlignPipeLine;

    private TankDriveCommand tankDriveCommand;
    private AlignRobotVisionCommand alignRobotCommand;

    @Override
    public void assign() {
        driveTrain = new DriveTrainSubsystem();
        vision = new VisionSubsystem();

        powerShootsAlignPipeLine = new BluePowerShootsAlignPipeLine();

        tankDriveCommand = new TankDriveCommand(driveTrain, () -> -gamepad1.left_stick_y * (gamepad2.left_trigger > 0.5 ? 1 : 0.5), () -> -gamepad1.right_stick_y * (gamepad2.left_trigger > 0.5 ? 1 : 0.5));
        alignRobotCommand = new AlignRobotVisionCommand(driveTrain, vision);

        vision.setAlignPipeLine(powerShootsAlignPipeLine);

        driveTrain.setDefaultCommand(tankDriveCommand);

        assign_buttons();
    }

    public void assign_buttons() {
        gp1.dpad_left().whenPressed(new InstantCommand(() -> powerShootsAlignPipeLine.target = powerShootsAlignPipeLine.target.left()));
        gp1.dpad_left().whenPressed(new InstantCommand(() -> powerShootsAlignPipeLine.target = powerShootsAlignPipeLine.target.right()));

        gp1.y().whenHeld(alignRobotCommand);
    }
}
