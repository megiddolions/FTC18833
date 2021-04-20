package org.firstinspires.ftc.teamcode.vison;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.Util;

@TeleOp(name="Test Lift Servos")
public class servoTest extends OpMode {

    Servo left;
    Servo right;

    @Override
    public void init() {
        left = hardwareMap.servo.get("LeftLift");
        right = hardwareMap.servo.get("RightLift");

        left.resetDeviceConfigurationForOpMode();
        right.resetDeviceConfigurationForOpMode();

        right.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if (gamepad1.a)
            set(1);
        else if (gamepad1.b)
            set(0.42);
    }

    private void set(double position) {
        left.setPosition(Util.clamp(position, 1, 0.42)); // Mechanical minimum DON'T change!
        right.setPosition(Util.clamp(position, 1, 0.42));
    }

    private double get_lift_position() {
        return left.getPosition();
    }
}
