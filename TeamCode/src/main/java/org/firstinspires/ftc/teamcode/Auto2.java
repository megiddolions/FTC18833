package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.commandftc.opModes.CommandBasedAuto;
import org.commandftc.opModes.LinearOpModeWithCommands;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.AlignRobotVisionCommand;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.SetVisionTargetCommand;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.autonomus.DriveForwardDistanceCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobellSubsystem;
import org.firstinspires.ftc.teamcode.vison.VisionTarget;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

@Autonomous(name = "ÂùŤő floor(∑1/n!)")
//@Autonomous(name = "ÂùŤő floor(∑1/n!)", preselectTeleOp="Drive for Robotosh")
public class Auto2 extends CommandBasedAuto {
    protected DriveTrainSubsystem driveTrain;
    protected ShooterSubsystem shooter;
    protected IntakeSubsystem intake;
    protected StorageSubSystem storage;
    protected WobellSubsystem wobellSubsystem;
    protected VisionSubsystem vision;

    protected AlignRobotVisionCommand alignWobellCommand;

    @Override
    public void plan() {
        driveTrain = new DriveTrainSubsystem();
        shooter = new ShooterSubsystem();
        intake = new IntakeSubsystem();
        storage = new StorageSubSystem();
        wobellSubsystem = new WobellSubsystem();
        vision = new VisionSubsystem();

        alignWobellCommand = new AlignRobotVisionCommand(driveTrain, vision);

        vision.setTarget(VisionTarget.BluePowerShoots);

        telemetry.addData("Runtime", this::getRuntime);
        telemetry.addData("Vision pipeline ms", vision.camera::getPipelineTimeMs);
        telemetry.addData("Vision error", vision::getError);
        telemetry.addData("align active", alignWobellCommand::isScheduled);
    }

    @Override
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
                alignWobellCommand,
                new SetVisionTargetCommand(vision, VisionTarget.BluePowerShoots),
                new AlignRobotVisionCommand(driveTrain, vision)
        );
    }
}
