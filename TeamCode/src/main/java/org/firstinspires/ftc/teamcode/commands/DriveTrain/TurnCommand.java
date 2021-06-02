package org.firstinspires.ftc.teamcode.commands.DriveTrain;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnCommand extends CommandBase {
    private final DriveTrainSubsystem driveTrain;
    private final double heading;

    public TurnCommand(DriveTrainSubsystem driveTrain, double heading) {
        this.driveTrain = driveTrain;
        this.heading = heading;
    }

    @Override
    public void initialize() {
        driveTrain.turnAsync(heading);
    }

    @Override
    public boolean isFinished() {
        return driveTrain.isBusy();
    }
}
