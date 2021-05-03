package org.firstinspires.ftc.teamcode.commands.DriveTrain;

import org.commandftc.Command;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class AlignRobotVisionCommand extends Command {
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
    protected void end() {
        driveTrain.stop();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(vision.getError()+offset) <= 2;
    }
}
