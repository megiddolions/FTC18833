package org.firstinspires.ftc.teamcode.commands.DriveTrain;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveToPositionCommand extends CommandBase {
    private final DriveTrainSubsystem driveTrain;
    private final Pose2d target;
    private final double speed;

    public DriveToPositionCommand(DriveTrainSubsystem driveTrain, Pose2d target, double speed) {
        this.driveTrain = driveTrain;
        this.target = target;
        this.speed = speed;

        addRequirements(driveTrain);
    }

    public DriveToPositionCommand(DriveTrainSubsystem driveTrain, Pose2d target) {
        this(driveTrain, target, 1);
    }

    @Override
    public void execute() {
        Pose2d difference = target.relativeTo(driveTrain.getPosition());

        driveTrain.holonomicDrive(difference.getX(), difference.getY(), difference.getRotation().getRadians(), speed);

    }

    @Override
    public boolean isFinished() {
        return getDistanceToTarget() < 0.1 && getAngleDistanceToTarget() < 0.1;
    }

    private double getDistanceToTarget() {
        return driveTrain.getPosition().getTranslation().getDistance(target.getTranslation());
    }

    private double getAngleDistanceToTarget() {
        return Math.abs(driveTrain.getPosition().getRotation().minus(target.getRotation()).getRadians());
    }
}
