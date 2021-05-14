package org.firstinspires.ftc.teamcode.commands.DriveTrain.autonomus;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static org.commandftc.RobotUniversal.telemetry;

public class DriveForwardDistanceCommand extends CommandBase {
    private final DriveTrainSubsystem driveTrain;
    private final double distance;

    public DriveForwardDistanceCommand(DriveTrainSubsystem driveTrain, double mm) {
        this.driveTrain = driveTrain;
        this.distance = mm;

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        driveTrain.set_for_autonomous();
        driveTrain.driveForwardDistance(distance);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stop();
    }

    @Override
    public boolean isFinished() {
        return !driveTrain.isBusy();
    }
}
