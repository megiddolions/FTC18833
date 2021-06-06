package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.commandftc.opModes.CommandBasedAuto;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.AlignPowerShootsCommand;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.AlignWobbleVisionCommand;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.HorizontalAlignCommand;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.ReturnToStartCommand;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.Shooter.SetShooterSpeedCommand;
import org.firstinspires.ftc.teamcode.commands.Shooter.WaitForShooterCommand;
import org.firstinspires.ftc.teamcode.commands.Storage.AutomaticStorageCommand;
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
import org.firstinspires.ftc.teamcode.vison.pipelines.align.NonePipeLine;
import org.firstinspires.ftc.teamcode.vison.pipelines.align.RingAlignPipeLine;
import org.firstinspires.ftc.teamcode.vison.pipelines.align.VisionTarget;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

@TeleOp(name = "ÂùŤő floor(∑1/n!)")
//@Autonomous(name = "ÂùŤő floor(∑1/n!)", preselectTeleOp="Drive for Robotosh")
public class Auto2 extends CommandBasedAuto {
    protected DriveTrainSubsystem driveTrain;
    protected ShooterSubsystem shooter;
    protected IntakeSubsystem intake;
    protected StorageSubSystem storage;
    protected WobbleSubsystem wobbleSubsystem;
    protected VisionSubsystem vision;

    protected BluePowerShootsAlignPipeLine powerShootsAlign;

    protected AlignPowerShootsCommand alignRobot;

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

        powerShootsAlign = new BluePowerShootsAlignPipeLine();

        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveTrain.setPoseEstimate(new Pose2d(-1.5438, 0.43463692038495186, Math.toRadians(180)));
//        driveTrain.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        alignRobot = new AlignPowerShootsCommand(driveTrain, vision);

        startShooter = new SetShooterSpeedCommand(shooter, 0.5);
        stopShooter = new SetShooterSpeedCommand(shooter, 0);

        index_ring = new IndexOneRingCommand(storage);

        wobbleSubsystem.setDefaultCommand(new WobellTargetPositionCommand(wobbleSubsystem));

        wobbleSubsystem.open();

        vision.set_for_autonomous();
        shooter.setLift(0.27);

        telemetry.addData("Runtime", this::getRuntime);
        telemetry.addData("Vision rear(ms)", vision.rearCamera::getPipelineTimeMs);
        telemetry.addData("Vision front(ms)", vision.frontCamera::getPipelineTimeMs);
        telemetry.addData("Vision rear error", vision::getError);
        telemetry.addData("Vision front error", vision::getFrontError);
        telemetry.addData("align active", alignRobot::isScheduled);
        telemetry.addData("pos", driveTrain::getPoseEstimate);
//        telemetry.addData("left", shooter::getLeftVelocity);

        TelemetryPacket init_telemetry_packet = new TelemetryPacket();
        DashboardUtil.drawRobot(init_telemetry_packet.fieldOverlay(), driveTrain.getPoseEstimate());
        FtcDashboard.getInstance().sendTelemetryPacket(init_telemetry_packet);
    }

    @Override
    public Command getAutonomousCommand() {
        int rings = vision.count_rings();
        telemetry.addLine("rings: " + rings);
        FtcDashboard.getInstance().getTelemetry().addLine("rings: " + rings);
        FtcDashboard.getInstance().getTelemetry().addLine("pixels: " + vision.getOrangePixels());
        FtcDashboard.getInstance().getTelemetry().update();
        vision.setAlignPipeLine(powerShootsAlign);

        Trajectory first_power_shoot_trajectory = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate(), true)
                .back(.3)
                .splineTo(new Vector2d(0, .30), Math.toRadians(-4))
                .build();

        return new SequentialCommandGroup(
                new InstantCommand(() -> powerShootsAlign.target = VisionTarget.PowerShoot.Right),
                startShooter,
                follow(first_power_shoot_trajectory),
                new WaitForShooterCommand(shooter, 2500),
                alignRobot,
                new InstantCommand(() -> powerShootsAlign.target = VisionTarget.PowerShoot.Center),
//                new WaitCommand(2),
                index_ring,
                new TurnCommand(driveTrain, Math.toRadians(3)),
//                alignRobot,
                new InstantCommand(() -> powerShootsAlign.target = VisionTarget.PowerShoot.Left),
                new WaitCommand(0.2),
                index_ring,
                new TurnCommand(driveTrain, Math.toRadians(5)),
//                alignRobot,
                new WaitCommand(0.2),
                index_ring,
                stopShooter,
                new InstantCommand(() -> {
                    vision.rearCamera.setPipeline(new NonePipeLine());
                    vision.front_pipeline = new RingAlignPipeLine();
                    vision.frontCamera.setPipeline(vision.front_pipeline);
                }),
                getRingCommand(rings, first_power_shoot_trajectory.end()),
                new WaitCommand(5),
                new ReturnToStartCommand(driveTrain, new Pose2d(-1.1, 0.40463692038495186, Math.toRadians(180)))
        );
    }

    private Command getRingCommand(int rings, Pose2d end) {
        switch (rings) {
            case 0:
                return getACommand(end);
            case 1:
                return getBCommand(end);
            case 4:
                return getCCommand(end);
            default:
                return new InstantCommand();
        }
    }

    private Command getACommand(Pose2d end) {
        Trajectory first_wobell_trajectory = driveTrain.trajectoryBuilder(end, true)
                .splineTo(new Vector2d(0.22, 1.2), Math.toRadians(90))
                .build();

        Trajectory second_wobell_trajectory_part_1 = driveTrain.trajectoryBuilder(first_wobell_trajectory.end())
                .forward(0.4)
                .build();

        Trajectory second_wobell_trajectory_part_2 = driveTrain.trajectoryBuilder(second_wobell_trajectory_part_1.end().plus(new Pose2d(0, 0, Math.toRadians(90))), true)
                .splineTo(new Vector2d(-0.86, 1.17), Math.toRadians(180))
                .build();

        Trajectory put_second_wobell_trajectory = driveTrain.trajectoryBuilder(second_wobell_trajectory_part_2.end(), true)
                .splineTo(new Vector2d(0.22, 1.15), Math.toRadians(90))
                .build();

        Trajectory parking_trajectory = driveTrain.trajectoryBuilder(put_second_wobell_trajectory.end(), true)
                .forward(0.3)
                .build();

        return new SequentialCommandGroup(
                new InstantCommand(() -> wobbleSubsystem.setTargetPosition(4000)),
                follow(first_wobell_trajectory),
                new InstantCommand(() -> wobbleSubsystem.close()),
                new InstantCommand(() -> wobbleSubsystem.setTargetPosition(4800)),
                follow(second_wobell_trajectory_part_1),
                turn(Math.toRadians(90)),
                follow(second_wobell_trajectory_part_2),
                new InstantCommand(() -> wobbleSubsystem.open()),
                new WaitCommand(0.4),
                new InstantCommand(() -> wobbleSubsystem.setTargetPosition(4200)),
                follow(put_second_wobell_trajectory),
                new InstantCommand(() -> wobbleSubsystem.close()),
                follow(parking_trajectory)
        );
    }

    private Command getBCommand(Pose2d end) {
        Trajectory first_wobell_trajectory = driveTrain.trajectoryBuilder(end, true)
                .splineTo(new Vector2d(0.70, 0.80), Math.toRadians(45))
                .build();

        Trajectory go_to_index_position_trajectory = driveTrain.trajectoryBuilder(first_wobell_trajectory.end())
                .splineTo(new Vector2d(-0.14, 0.90), Math.toRadians(180))
                .build();

        Trajectory intake_trajectory = driveTrain.trajectoryBuilder(go_to_index_position_trajectory.end())
                .forward(.4)
                .build();

        Trajectory second_wobell_trajectory = driveTrain.trajectoryBuilder(intake_trajectory.end(), true)
                .splineTo(new Vector2d(-0.73, 1.1), Math.toRadians(180))
                .build();

        Trajectory pick_up_wobell_trajectory = driveTrain.trajectoryBuilder(second_wobell_trajectory.end(), true)
                .back(0.3)
                .build();

        Trajectory drop_second_wobell_trajectory = driveTrain.trajectoryBuilder(pick_up_wobell_trajectory.end(), true)
                .forward(0.4)
                .splineTo(new Vector2d(0.43, 0.83), Math.toRadians(180))
                .build();

        Trajectory parking_trajectory = driveTrain.trajectoryBuilder(drop_second_wobell_trajectory.end(), true)
                .forward(0.3)
                .build();

        return new SequentialCommandGroup(
                new InstantCommand(() -> shooter.setLift(0.26)),
                new InstantCommand(() -> wobbleSubsystem.setTargetPosition(4000)),
                follow(first_wobell_trajectory),
                new InstantCommand(() -> wobbleSubsystem.close()),
                new InstantCommand(() -> wobbleSubsystem.setTargetPosition(3000)),
                new SetShooterSpeedCommand(shooter, 0.55),
                follow(go_to_index_position_trajectory),
                new InstantCommand(() -> intake.intake(1)),
                new InstantCommand(() -> storage.index(1)),
                follow(intake_trajectory),
                wait(1.6),
                new InstantCommand(() -> intake.intake(0)),
                new InstantCommand(() -> storage.index(0)),
                new SetShooterSpeedCommand(shooter, 0),
                new InstantCommand(() -> wobbleSubsystem.setTargetPosition(5000)),
                follow(second_wobell_trajectory),
                follow(pick_up_wobell_trajectory),
                new InstantCommand(() -> wobbleSubsystem.open()),
                follow(drop_second_wobell_trajectory),
                new InstantCommand(() -> wobbleSubsystem.close()),
                follow(parking_trajectory)
        );
    }

    private Command getCCommand(Pose2d end) {
        Trajectory first_wobell_trajectory = driveTrain.trajectoryBuilder(end, true)
                .splineTo(new Vector2d(1.3, 1.5), Math.toRadians(45))
                .build();

        Trajectory go_to_index_position_trajectory = driveTrain.trajectoryBuilder(first_wobell_trajectory.end())
                .splineTo(new Vector2d(-0.20, 0.92), Math.toRadians(183))
                .build();

        Trajectory score__first_ring_trajectory = driveTrain.trajectoryBuilder(go_to_index_position_trajectory.end(), DriveTrainSubsystem.getVelocityConstraint(0.1, Math.toRadians(165), 0.28))
//                .back(0.1)
                .forward(0.2)
                .build();

        Trajectory store_other_rings_trajectory = driveTrain.trajectoryBuilder(score__first_ring_trajectory.end())
                .forward(0.6)
                .build();

        Trajectory score_other_rings_trajectory = driveTrain.trajectoryBuilder(store_other_rings_trajectory.end(), true)
                .splineTo(new Vector2d(0, 0.9), 180)
                .build();

        Trajectory drop_second_wobble_trajectory = driveTrain.trajectoryBuilder(score_other_rings_trajectory.end(), true)
                .splineTo(new Vector2d(1.1, 1.8), Math.toRadians(0))
                .build();

        Trajectory parking_trajectory = driveTrain.trajectoryBuilder(drop_second_wobble_trajectory.end())
                .splineTo(new Vector2d(0.2, 0.9), Math.toRadians(-135))
                .build();

        return new SequentialCommandGroup(
                new InstantCommand(() -> shooter.setLift(0.35)),
                new InstantCommand(() -> wobbleSubsystem.setTargetPosition(4000)),
                follow(first_wobell_trajectory),
                new InstantCommand(() -> wobbleSubsystem.close()),
                new InstantCommand(() -> wobbleSubsystem.setTargetPosition(3000)),
                new InstantCommand(() -> shooter.setLift(0.2)),
                new InstantCommand(() -> shooter.setPower(0.55)),
                follow(go_to_index_position_trajectory),
                new HorizontalAlignCommand(driveTrain, vision),
                new InstantCommand(() -> {
                    vision.front_pipeline = new NonePipeLine();
                    vision.setTarget(VisionTarget.BlueWobble);
                    vision.frontCamera.setPipeline(vision.front_pipeline);
                }),
                new InstantCommand(() -> driveTrain.setPoseEstimate(new Pose2d(driveTrain.getPoseEstimate().getX(), 0.93, Math.toRadians(180)))),
                driveForward(0.12, 1),
                new InstantCommand(() -> {storage.index(1);intake.intake(1);}),
                follow(score__first_ring_trajectory),
                new WaitCommand(2.5),
                new InstantCommand(() -> {storage.index(0); shooter.setPower(0);}),
                new InstantCommand(() -> storage.setDefaultCommand(new AutomaticStorageCommand(storage))),
//                follow(store_other_rings_trajectory),
                driveForward(0.6, 0.2),
                turn(Math.toRadians(135)),
                new WaitCommand(2),
                new InstantCommand(() -> wobbleSubsystem.setTargetPosition(5000)),
                new WaitCommand(0.5),
                new AlignWobbleVisionCommand(driveTrain, vision),
                driveForward(0.2, -0.4),
                new InstantCommand(() -> wobbleSubsystem.open()),
                follow(score_other_rings_trajectory),
                new WaitCommand(2),
                follow(drop_second_wobble_trajectory),
                new InstantCommand(() -> wobbleSubsystem.close()),
                new InstantCommand(() -> wobbleSubsystem.setTargetPosition(0)),
                follow(parking_trajectory)
        );
    }

    private Command follow(Trajectory trajectory) {
        return new FollowTrajectoryCommand(driveTrain, trajectory);
    }

    private Command turn(double angle) {
        return new TurnCommand(driveTrain, angle);
    }

    private Command wait(int seconds) {
        return new WaitCommand(seconds);
    }

    private Command wait(double seconds) {
        return new WaitCommand(seconds);
    }

    private Command driveForward(double distance, double speed) {
        return new CommandBase() {

            private Pose2d start;
            @Override
            public void initialize() {
                driveTrain.setPower(speed);
                start = driveTrain.getPoseEstimate();
            }

            @Override
            public boolean isFinished() {
                return driveTrain.getPoseEstimate().vec().distTo(start.vec()) >= distance;
            }

            @Override
            public void end(boolean interrupted) {
                driveTrain.stop();
            }
        };
    }
}
