package org.commandftc.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.commandftc.CommandScheduler;
import org.commandftc.Gp;
import org.commandftc.RobotUniversal;
import org.commandftc.Subsystem;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

public abstract class CommandBasedTeleOp extends OpMode {

    private Set<Subsystem> subsystems = new HashSet<>();

    protected Gp gp1 = new Gp(gamepad1);
    protected Gp gp2 = new Gp(gamepad2);


    protected final void addSubsystems(Subsystem ... sss) {
        Collections.addAll(subsystems, sss);
    }

    private void registerSubsystems() {
        for(Subsystem ss : subsystems) {
            CommandScheduler.registerSubsystem(ss);
        }
    }

    /**
     * DON'T OVERRIDE THIS! IT CALLS init_impl() (WHICH YOU SHOULD INSTEAD OVERRIDE)
     * AND DOES SOMETHING ELSE!!!!!
     */
    @Override
    public final void init() {
        RobotUniversal.hardwareMap = hardwareMap;
        RobotUniversal.telemetry = telemetry;
        RobotUniversal.opMode = this;
        assign();
        registerSubsystems();
    }

    public abstract void assign();

    @Override
    public final void loop() {
        CommandScheduler.setOpModeActive(true);
        CommandScheduler.runOnce();
        telemetry.update();
    }

    @Override
    public final void stop() {
        CommandScheduler.setOpModeActive(false);
        CommandScheduler.unscheduleAll();
        CommandScheduler.unregisterAllButtons();
        CommandScheduler.unregisterAllSubsystems();
    }
}
