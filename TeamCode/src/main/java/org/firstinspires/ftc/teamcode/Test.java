package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Test")
public class Test extends OpMode {
    DcMotor motor;
    @Override
    public void init() {
        motor = hardwareMap.dcMotor.get("RearRight");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Pos", motor::getCurrentPosition);
    }

    @Override
    public void loop() {
        motor.setPower(-gamepad1.left_stick_y);
        telemetry.update();
    }
}
