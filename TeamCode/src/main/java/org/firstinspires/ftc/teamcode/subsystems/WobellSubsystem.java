package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.commandftc.RobotUniversal.*;

public class WobellSubsystem extends SubsystemBase {
    private final CRServo leftLift;
    private final CRServo rightLift;
    private final Servo middle;
    private final IntSupplier encoder;
    private int target_position;

    public WobellSubsystem() {
        leftLift = hardwareMap.crservo.get("LeftWobellLift");
        rightLift = hardwareMap.crservo.get("RightWobellLift");
        middle = hardwareMap.servo.get("WobellServo");

        DcMotor encoder = hardwareMap.dcMotor.get("IntakeMotor");
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.encoder = encoder::getCurrentPosition;

        rightLift.setDirection(CRServo.Direction.REVERSE);
        leftLift.setDirection(CRServo.Direction.FORWARD);

        setTargetPosition(getTargetPosition());
    }

//    @Override
//    public void periodic() {
//        if (auto) {
//            setLift((getTargetPosition() - getCurrentPosition()) / -2000.0);
//        }
//    }

    public boolean isBusy() {
        return Math.abs(getTargetPosition() - getCurrentPosition()) >= 200;
    }

    public int getCurrentPosition() {
        return encoder.getAsInt();
    }

    public int getTargetPosition() {
        return target_position;
    }

    public void setTargetPosition(int target_position) {
        this.target_position = target_position;
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

    public double getPower() {
        return leftLift.getPower();
    }
}
