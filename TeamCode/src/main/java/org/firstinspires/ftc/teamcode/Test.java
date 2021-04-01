package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;

@TeleOp(name = "V")
public class Test extends OpMode {
    private DcMotorEx motor;
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "LeftShooter");
        motor.setMotorEnable();
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
//        motor.setPower(-gamepad1.left_stick_y);
//        motor.setVelocity(Util.RPM_to_ticks_per_second(120));
        motor.setVelocity(ShooterConstants.RPM_to_ticks_per_second(171));
        telemetry.addData("v", ShooterConstants.ticks_per_second_to_RPM(80));
        telemetry.addData("p", motor.getPower());
        telemetry.update();
    }
}
