package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.lib.CommandBaseOpMode;
import org.firstinspires.ftc.teamcode.lib.SubsystemBase;

import java.util.HashMap;
import java.util.Map;

public class Robot {
    public static CommandBaseOpMode opMode;
    public final static Map<String, SubsystemBase> subsystems = new HashMap<>();

    public static CommandBaseOpMode OpMode() {
        return opMode;
    }

    public static void addSubsystem(SubsystemBase subsystem) {
        subsystems.put(subsystem.name(), subsystem);
    }

    public static void update_subsystems() {
        for (Map.Entry<String, SubsystemBase> entry : subsystems.entrySet()) {
            entry.getValue().periodic();
        }
    }

    public static void clearSubsystems() {
        subsystems.clear();
    }

    public static void init(CommandBaseOpMode opMode) {
        Robot.opMode = opMode;
    }
}
