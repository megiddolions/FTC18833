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
    private double target = -Double.MAX_VALUE;

    public AlignRobotVuforiaCommand(DriveTrainSubsystem driveTrain, VuforiaSubsystem vuforia) {
        this.driveTrain = driveTrain;
        this.vuforia = vuforia;
        pid = new PIDController(0.01, 0.00001, 0.000002);

        addRequirements(driveTrain, vuforia);
    }

    @Override
    public void execute() {
        if (vuforia.has_target()) {
            double spin = pid.output(vuforia.heading() - 4, 0);
            driveTrain.setPower(spin, -spin);
        } else {
            driveTrain.stop();
        }
        telemetry.addData("target", target);
    }

    @Override
    protected void end() {
        pid.clear();
        target = -Double.MAX_VALUE;
    }

    @Override
    public boolean isFinished() {
        return target != -Double.MAX_VALUE && Math.abs(target - driveTrain.getHeading().getDegrees()) <= 2;
    }
}
