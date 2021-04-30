package org.firstinspires.ftc.teamcode.commands.Util;

import org.commandftc.Command;

import java.io.DataOutput;

import static org.commandftc.RobotUniversal.*;

public class WaitCommand extends Command {
    private final double time;
    private double start = Double.MAX_VALUE;
    private boolean has_started = true;

    public WaitCommand(double time) {
        this.time = time;
    }

    @Override
    public void init() {
        start = opMode.getRuntime();
//        has_started = true;
    }

    @Override
    public void execute() {
        telemetry.addData("Command", this.getClass().getSimpleName());
    }

    @Override
    protected void end() {
//        has_started = false;
        start = Double.MAX_VALUE;
    }

    @Override
    public boolean isFinished() {
        return has_started && start <= opMode.getRuntime() - time;
    }
}
