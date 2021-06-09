package org.firstinspires.ftc.teamcode.opmods.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.commandftc.opModes.CommandBasedAuto;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.Storage.IndexDistanceCommand;
import org.firstinspires.ftc.teamcode.commands.Wobell.WobellTargetPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public abstract class DefaultAuto extends CommandBasedAuto {
    protected DriveTrainSubsystem driveTrain;
    protected ShooterSubsystem shooter;
    protected StorageSubSystem storage;
    protected IntakeSubsystem intake;
    protected VisionSubsystem vision;
    protected WobbleSubsystem wobbleSubsystem;

    @Override
    public final void plan() {
        driveTrain = new DriveTrainSubsystem();
        shooter = new ShooterSubsystem();
        storage = new StorageSubSystem();
        intake = new IntakeSubsystem();
        vision = new VisionSubsystem();
        wobbleSubsystem = new WobbleSubsystem();

        vision.set_for_autonomous();
        wobbleSubsystem.setDefaultCommand(new WobellTargetPositionCommand(wobbleSubsystem));

        init_robot();
    }

    protected abstract void init_robot();

    protected TrajectoryBuilder trajectoryBuilder(Pose2d last_trajectory_end) {
        return driveTrain.trajectoryBuilder(last_trajectory_end);
    }

    protected TrajectoryBuilder trajectoryBuilder(Pose2d last_trajectory_end, boolean reversed) {
        return driveTrain.trajectoryBuilder(last_trajectory_end, reversed);
    }

    protected Command follow(Trajectory trajectory) {
        return new FollowTrajectoryCommand(driveTrain, trajectory);
    }

    protected Command turn(double angle) {
        return new TurnCommand(driveTrain, angle);
    }

    protected Command wait(int seconds) {
        return new WaitCommand(seconds);
    }

    protected Command wait(double seconds) {
        return new WaitCommand(seconds);
    }

    protected Command driveForward(double distance, double speed) {
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

    protected Command index_distance(double mm) {
        return new IndexDistanceCommand(storage, mm);
    }
}
