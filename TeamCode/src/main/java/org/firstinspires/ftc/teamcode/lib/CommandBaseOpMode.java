package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;

public abstract class CommandBaseOpMode extends OpMode {

    public void update() {
        Robot.update_subsystems();
    }

    @Override
    public void stop() {
        Robot.getInstance().clearSubsystems();
    }
}
