package org.firstinspires.ftc.teamcode.commands.DriveTrain;

import org.commandftc.Command;
import org.firstinspires.ftc.teamcode.lib.PIDController;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VuforiaSubsystem;

import static org.commandftc.RobotUniversal.*;

public class AlignRobotVuforiaCommand extends Command {
    private final VuforiaSubsystem vuforia;
    private final DriveTrainSubsystem driveTrain;
    private final PIDController pid;
    double last = 0;
    double time = 0;

    public AlignRobotVuforiaCommand(DriveTrainSubsystem driveTrain, VuforiaSubsystem vuforia) {
        this.driveTrain = driveTrain;
        this.vuforia = vuforia;
        pid = new PIDController(0.01, 0.00001, 0.000002);

        addRequirements(driveTrain, vuforia);
    }

    @Override
    public void init() {
    }

    @Override
    public void execute() {
        last = time;
        time = opMode.getRuntime();
        if (vuforia.has_target()) {
            driveTrain.driveLeft(-vuforia.vertical_error());

        } else {
            driveTrain.stop();
        }
    }

    @Override
    protected void end() {
        pid.clear();
        driveTrain.stop();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(vuforia.vertical_error()) <= 0.2;
    }
}
