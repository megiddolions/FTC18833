package org.firstinspires.ftc.teamcode.commands.Util;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static org.commandftc.RobotUniversal.*;

public class LoopTimeCommand extends CommandBase {
    private double last_time;
    private double dt = 0;

    @Override
    public void initialize() {
        last_time = opMode.getRuntime();
        telemetry.addData("dt", () -> dt);
    }

    @Override
    public void execute() {
        double time = opMode.getRuntime();
        dt = time - last_time;
        last_time = time;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
