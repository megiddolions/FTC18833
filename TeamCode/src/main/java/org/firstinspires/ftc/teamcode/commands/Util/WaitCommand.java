package org.firstinspires.ftc.teamcode.commands.Util;

import org.commandftc.Command;

import java.io.DataOutput;

import static org.commandftc.RobotUniversal.*;

public class WaitCommand extends Command {
    private final double time;
    private double start = Double.MAX_VALUE;

    public WaitCommand(double ms) {
        this.time = ms * 1000;
    }

    @Override
    public void init() {
        start = opMode.getRuntime();
    }

    @Override
    protected void end() {
        start = Double.MAX_VALUE;
    }

    @Override
    public boolean isFinished() {
        return start <= opMode.getRuntime() - time;
    }
}
