package org.firstinspires.ftc.teamcode.lib;

import org.firstinspires.ftc.teamcode.Robot;

public abstract class SubsystemBase {
    public SubsystemBase() {
        Robot.addSubsystem(this);
    }

    public void periodic() {}

    public String name() {
        return this.getClass().getSimpleName();
    }
}
