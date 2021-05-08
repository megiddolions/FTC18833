package org.firstinspires.ftc.teamcode.commands.DriveTrain;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AlignRobotVisionCommand extends CommandBase {
    private final DriveTrainSubsystem driveTrain;
    private final VisionSubsystem vision;
    private final double offset;

    public AlignRobotVisionCommand(DriveTrainSubsystem driveTrain, VisionSubsystem vision) {
        this.driveTrain = driveTrain;
        this.vision = vision;
        this.offset = 0;

        addRequirements(driveTrain, vision);
    }

    public AlignRobotVisionCommand(DriveTrainSubsystem driveTrain, VisionSubsystem vision, double offset) {
        this.driveTrain = driveTrain;
        this.vision = vision;
        this.offset = offset;

        addRequirements(driveTrain, vision);
    }

    @Override
    public void execute() {
        driveTrain.driveLeft((-vision.getError()+offset) / 500);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stop();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(vision.getError()+offset) <= 2;
    }
}
