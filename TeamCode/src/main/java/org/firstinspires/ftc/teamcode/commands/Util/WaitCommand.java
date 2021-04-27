package org.firstinspires.ftc.teamcode.commands.Util;

import org.commandftc.Command;

import static org.commandftc.RobotUniversal.*;

public class WaitCommand extends Command {
    private final double time;
    private double start;

    public WaitCommand(double time) {
        this.time = time;
    }

    @Override
    public void init() {
        start = opMode.getRuntime();
    }

    @Override
    public void execute() {
        telemetry.addData("time", start);
    }

    @Override
    public boolean isFinished() {
        return start <= opMode.getRuntime() - time;
    }
}
