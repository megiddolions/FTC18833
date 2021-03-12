package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Encoder {
    private final DcMotor motor;

    public Encoder(DcMotor motor) {
//        if (motor == null) {
//            throw new Exception("DcMotor is null");
//        } else if (motor.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER ||
//            motor.getMode() == DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
//            throw new Exception("DcMotor has wrong run mode: " + motor.getMode().toString());
//        }
        this.motor = motor;
    }

    public DcMotor getMotor() {
        return motor;
    }

    public double get() {
        return motor.getCurrentPosition();
    }

    public void reset() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
