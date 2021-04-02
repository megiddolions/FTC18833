package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Robot {
    private Robot() {}
    private final static Robot instance = new Robot();
    public OpMode opMode;

    public static Robot getInstance() {
        return instance;
    }

    public void init(OpMode opMode) {
        this.opMode = opMode;
    }
}
