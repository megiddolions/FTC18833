package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.commandftc.opModes.CommandBasedAuto;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.AlignPowerShootsCommand;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.commands.Shooter.SetShooterSpeedCommand;
import org.firstinspires.ftc.teamcode.commands.Storage.IndexOneRingCommand;
import org.firstinspires.ftc.teamcode.commands.Wobell.WobellTargetPositionCommand;
import org.firstinspires.ftc.teamcode.lib.DashboardUtil;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;
import org.firstinspires.ftc.teamcode.vison.pipelines.align.BluePowerShootsAlignPipeLine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TestAuto extends CommandBasedAuto {
    protected DriveTrainSubsystem driveTrain;
    protected ShooterSubsystem shooter;
    protected IntakeSubsystem intake;
    protected StorageSubSystem storage;
    protected WobbleSubsystem wobbleSubsystem;
    protected VisionSubsystem vision;

    protected BluePowerShootsAlignPipeLine powerShootsPipeLine;

    protected AlignPowerShootsCommand alignWobellCommand;

    protected SetShooterSpeedCommand startShooter;
    protected SetShooterSpeedCommand stopShooter;

    protected IndexOneRingCommand index_ring;

    @Override
    public void plan() {
        driveTrain = new DriveTrainSubsystem();
        shooter = new ShooterSubsystem();
        intake = new IntakeSubsystem();
        storage = new StorageSubSystem();
        wobbleSubsystem = new WobbleSubsystem();
        vision = new VisionSubsystem();

        powerShootsPipeLine = new BluePowerShootsAlignPipeLine();

        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveTrain.setPoseEstimate(new Pose2d(-1.5438, 0.40463692038495186, Math.toRadians(180)));
//        driveTrain.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        alignWobellCommand = new AlignPowerShootsCommand(driveTrain, vision);

        startShooter = new SetShooterSpeedCommand(shooter, 0.5);
        stopShooter = new SetShooterSpeedCommand(shooter, 0);

        index_ring = new IndexOneRingCommand(storage);

        wobbleSubsystem.setDefaultCommand(new WobellTargetPositionCommand(wobbleSubsystem));

        wobbleSubsystem.open();

        vision.set_for_autonomous();
        shooter.setLift(0.27);

        telemetry.addData("Runtime", this::getRuntime);
        telemetry.addData("Vision pipeline ms", vision.rearCamera::getPipelineTimeMs);
        telemetry.addData("Vision error", vision::getError);
        telemetry.addData("align active", alignWobellCommand::isScheduled);
        telemetry.addData("pos", driveTrain::getPoseEstimate);

        TelemetryPacket init_telemetry_packet = new TelemetryPacket();
        DashboardUtil.drawRobot(init_telemetry_packet.fieldOverlay(), driveTrain.getPoseEstimate());
        FtcDashboard.getInstance().sendTelemetryPacket(init_telemetry_packet);

    }

    @Override
    public Command getAutonomousCommand() {
        Trajectory first_power_shoot_trajectory = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate(), true)
                .back(.3)
                .splineToConstantHeading(new Vector2d(0, 0), Math.toRadians(-4))
                .build();

        Trajectory second_power_shoot_trajectory = driveTrain.trajectoryBuilder(first_power_shoot_trajectory.end(), true)
                .splineToConstantHeading(new Vector2d(0, 0), 0)
                .build();

        return new SequentialCommandGroup(
                new FollowTrajectoryCommand(driveTrain, first_power_shoot_trajectory),
                new FollowTrajectoryCommand(driveTrain, second_power_shoot_trajectory)
        );
    }
}
