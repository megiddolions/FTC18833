package org.firstinspires.ftc.teamcode.commands.Util;

import org.commandftc.Command;

import static org.commandftc.RobotUniversal.*;

public class LoopTimeCommand extends Command {
    private double last_time;
    private double dt = 0;
    @Override
    public void init() {
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
