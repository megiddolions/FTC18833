package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.commandftc.opModes.CommandBasedAuto;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.AlignRobotVisionCommand;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.commands.Wobell.OpenWobellCommand;
import org.firstinspires.ftc.teamcode.commands.Wobell.WobellLiftCommand;
import org.firstinspires.ftc.teamcode.commands.Wobell.WobellTargetPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobellSubsystem;
import org.firstinspires.ftc.teamcode.vison.VisionTarget;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

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

        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveTrain.setPoseEstimate(new Pose2d(1.145, 0.2425, Math.toRadians(180)));
//        driveTrain.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        alignWobellCommand = new AlignRobotVisionCommand(driveTrain, vision);

        wobellSubsystem.setDefaultCommand(new WobellTargetPositionCommand(wobellSubsystem));

        wobellSubsystem.open();

        vision.setTarget(VisionTarget.BlueTower);

        telemetry.addData("Runtime", this::getRuntime);
        telemetry.addData("Vision pipeline ms", vision.camera::getPipelineTimeMs);
        telemetry.addData("Vision error", vision::getError);
        telemetry.addData("align active", alignWobellCommand::isScheduled);
        telemetry.addData("pos", driveTrain::getPoseEstimate);

        new CommandBase() {
            @Override
            public void execute() {
                FtcDashboard.getInstance().getTelemetry().addData("drive power", driveTrain.getFrontRightPower());
                FtcDashboard.getInstance().getTelemetry().update();
            }
        }.schedule();
    }

    @Override
    public Command getAutonomousCommand() {
        Trajectory trajectory1 = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                .forward(-2.6)
                .build();

        Trajectory trajectory2 = driveTrain.trajectoryBuilder(trajectory1.end())
                .forward(0.7)
                .build();

        return new SequentialCommandGroup(
                new FollowTrajectoryCommand(driveTrain, trajectory1),
                new InstantCommand(() -> wobellSubsystem.setTargetPosition(4000)),
                new WaitCommand(2),
                new InstantCommand(() -> wobellSubsystem.close()),
        new InstantCommand(() -> wobellSubsystem.setTargetPosition(0)),
                new WaitCommand(2),
                new FollowTrajectoryCommand(driveTrain, trajectory2)
        );
    }
}
