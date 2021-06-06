package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.commandftc.opModes.CommandBasedTeleOp;

@TeleOp(name="TestServo", group = "tests")
public class ServoTest extends CommandBasedTeleOp {
    protected Servo servo;

    @Override
    public void assign() {
        servo = hardwareMap.servo.get("RightWobellServo");

        servo.setPosition(0.5);

        gp1.dpad_right().whenPressed(() -> servo.setPosition(servo.getPosition() + 0.1));
        gp1.dpad_left().whenPressed(() -> servo.setPosition(servo.getPosition() - 0.1));

        gp1.a().whenPressed(() -> servo.setPosition(1));
        gp1.b().whenPressed(() -> servo.setPosition(0));

        telemetry.addData("pos", servo::getPosition);
    }
}
