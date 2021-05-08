package org.commandftc.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.commandftc.RobotUniversal;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public abstract class CommandBasedAuto extends OpMode {
    private Command autonomousCommand;
    /**
     * DON'T OVERRIDE THIS! IT CALLS init_impl() (WHICH YOU SHOULD INSTEAD OVERRIDE)
     * AND DOES SOMETHING ELSE!!!!!
     */
    @Override
    public final void init() {
        RobotUniversal.hardwareMap = hardwareMap;
        RobotUniversal.telemetry = telemetry;
        RobotUniversal.opMode = this;
        plan();
    }

    public final void setAutonomousCommand(Command command) {
        this.autonomousCommand = command;
    }

    @Override
    public final void start() {
        autonomousCommand.schedule();
    }

    public abstract void plan();

    @Override
    public final void loop() {
        CommandScheduler.getInstance().run();
        telemetry.update();
    }

    @Override
    public final void stop() {
        CommandScheduler.getInstance().close();
    }
}
