package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.lib.CommandBaseOpMode;
import org.firstinspires.ftc.teamcode.lib.SubsystemBase;

import java.util.List;
import java.util.Vector;

public class Robot {
    private Robot() {
        subsystems = new Vector<>();
    }
    private final static Robot instance = new Robot();
    public CommandBaseOpMode opMode;
    private final List<SubsystemBase> subsystems;

    public static Robot getInstance() {
        return instance;
    }

    public static CommandBaseOpMode OpMode() {
        return instance.opMode;
    }

    public static void addSubsystem(SubsystemBase subsystem) {
        instance.subsystems.add(subsystem);
    }

    public static void update_subsystems() {
        for (SubsystemBase subsystem : instance.subsystems) {
            subsystem.periodic();
        }
    }

    public void clearSubsystems() {
        subsystems.clear();
    }

    public void init(CommandBaseOpMode opMode) {
        this.opMode = opMode;
    }
}
