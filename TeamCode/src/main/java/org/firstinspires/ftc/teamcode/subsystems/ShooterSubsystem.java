package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.commandftc.Subsystem;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.lib.Util;

import static org.commandftc.RobotUniversal.hardwareMap;

public class ShooterSubsystem extends Subsystem {
    private final DcMotorEx left;
    private final DcMotorEx right;
    private final Servo leftLift;
    private final Servo rightLift;
    public PIDFCoefficients pid;

    public ShooterSubsystem() {
        left = hardwareMap.get(DcMotorEx.class, ShooterConstants.kLeftShooterName);
        right = hardwareMap.get(DcMotorEx.class, ShooterConstants.kRightShooterName);

        leftLift = hardwareMap.servo.get("LeftLift");
        rightLift = hardwareMap.servo.get("RightLift");

//        setLift(0);

        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        rightLift.setDirection(Servo.Direction.REVERSE);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pid = left.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setPower(double power) {
        left.setPower(power);
        right.setPower(power);
    }

    public double getPower() {
        return left.getPower();
    }

    public void toggle(double power) {
        if (getPower() == 0) {
            setPower(power);
        } else {
            setPower(0);
        }
    }



    public void update_toggle(double power) {
        if (getPower() == 0) {
            setPower(power);
        }
    }

    public void setVelocity(double RPM) {
        left.setVelocity(ShooterConstants.RPM_to_ticks_per_second(RPM));
        right.setVelocity(ShooterConstants.RPM_to_ticks_per_second(RPM));
    }

    public double getLeftVelocity() {
        return ShooterConstants.ticks_per_second_to_RPM(left.getVelocity());
    }
    
    public double getRightVelocity() {
        return ShooterConstants.ticks_per_second_to_RPM(right.getVelocity());
    }

    public int get_left_encoder() {
        return left.getCurrentPosition();
    }

    public int get_right_encoder() {
        return right.getCurrentPosition();
    }

    public void setLift(double position) {
        // Mechanical minimum and maximum of system
        final double min = 0.20;
        final double max = 0.55;
        leftLift.setPosition(Util.clamp(position, max, min));
        rightLift.setPosition(Util.clamp(position, max, min));
    }

    public double getLift() {
        return leftLift.getPosition();
    }
}