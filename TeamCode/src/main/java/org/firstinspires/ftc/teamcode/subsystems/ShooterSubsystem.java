package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.lib.SubsystemBase;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final DcMotorEx left;
    private final DcMotorEx right;
    private final DcMotorEx indexer;
    public PIDFCoefficients pid;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        left = hardwareMap.get(DcMotorEx.class, ShooterConstants.kLeftShooterName);
        right = hardwareMap.get(DcMotorEx.class, ShooterConstants.kRightShooterName);
        indexer = hardwareMap.get(DcMotorEx.class, ShooterConstants.kIndexShooterName);

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.FORWARD);

        indexer.setDirection(DcMotorSimple.Direction.FORWARD);

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
}