package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final DcMotorEx left;
    private final DcMotorEx right;
    private final DcMotor indexer;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        left = hardwareMap.get(DcMotorEx.class, "LeftShooter");
        right = hardwareMap.get(DcMotorEx.class, "RightShooter");
        indexer = hardwareMap.dcMotor.get("IndexMotor");

        right.setDirection(DcMotorSimple.Direction.REVERSE);

        indexer.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setPower(double power) {
        left.setPower(power);
        right.setPower(power);
    }

    public double getPower() {
        return left.getPower();
    }

    public void index(double power) {
        indexer.setPower(power);
    }

    public void toggle(double power) {
        if (getPower() == 0) {
            setPower(power);
        } else {
            setPower(0);
        }
    }

    public void toggle_index(double power) {
        if (indexer.getPower() == 0) {
            indexer.setPower(power);
        } else {
            indexer.setPower(0);
        }
    }

    public void setVelocity(double velocity) {
        left.setVelocity(velocity);
        right.setVelocity(velocity);
    }

    public double getLeftVelocity() {
        return left.getVelocity();
    }
    
    public double getRightVelocity() {
        return right.getVelocity();
    }


    public int get_left_encoder() {
        return left.getCurrentPosition();
    }

    public int get_right_encoder() {
        return right.getCurrentPosition();
    }
}