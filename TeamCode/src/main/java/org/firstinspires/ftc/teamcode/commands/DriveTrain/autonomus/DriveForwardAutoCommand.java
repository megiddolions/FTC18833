package org.firstinspires.ftc.teamcode.commands.DriveTrain.autonomus;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static org.commandftc.RobotUniversal.telemetry;

public class DriveForwardAutoCommand extends CommandBase {
    private final DriveTrainSubsystem driveTrain;
    private final double distance;
    private boolean has_started = false;

    public DriveForwardAutoCommand(DriveTrainSubsystem driveTrain, double mm) {
        this.driveTrain = driveTrain;
        this.distance = mm;

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        driveTrain.driveForwardDistance(distance);
        has_started = true;
    }

    @Override
    public void execute() {
        telemetry.addData("Command", this.getClass().getSimpleName());
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stop();
        has_started = false;
    }

    @Override
    public boolean isFinished() {
        return has_started && !driveTrain.isBusy();
    }
}
