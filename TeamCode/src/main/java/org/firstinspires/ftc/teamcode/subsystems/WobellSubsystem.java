package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.commandftc.Subsystem;

import static org.commandftc.RobotUniversal.*;

public class WobellSubsystem extends Subsystem {
    private final CRServo leftLift;
    private final CRServo rightLift;
    private final Servo middle;

    public WobellSubsystem() {
        leftLift = hardwareMap.crservo.get("LeftWobellLift");
        rightLift = hardwareMap.crservo.get("RightWobellLift");
        middle = hardwareMap.servo.get("WobellServo");

        rightLift.setDirection(CRServo.Direction.REVERSE);
        leftLift.setDirection(CRServo.Direction.FORWARD);

//        setLift(0.5);
    }

    public void open() {
        middle.setPosition(0);
    }

    public void close() {
        middle.setPosition(1);
    }

    public void setLift(double power) {
        leftLift.setPower(power);
        rightLift.setPower(power);
    }
}
