package org.firstinspires.ftc.teamcode.commands.DriveTrain;

import org.commandftc.Command;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import static org.commandftc.RobotUniversal.*;

public class AlignRobotVisionCommand extends Command {
    private final DriveTrainSubsystem driveTrain;
    private final VisionSubsystem vision;

    public AlignRobotVisionCommand(DriveTrainSubsystem driveTrain, VisionSubsystem vision) {
        this.driveTrain = driveTrain;
        this.vision = vision;

//        telemetry.addData("error", () -> vision.getError() / 500);

        addRequirements(driveTrain, vision);
    }

    @Override
    public void execute() {
        driveTrain.driveLeft(-vision.getError() / 500);
        telemetry.addData("error", vision.getError() / 500);
        telemetry.update();
    }

    @Override
    protected void end() {
        driveTrain.stop();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(vision.getError()) <= 2;
    }
}
