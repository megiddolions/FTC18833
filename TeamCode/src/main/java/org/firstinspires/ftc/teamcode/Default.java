package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public abstract class Default extends OpMode {

    protected DcMotor rearLeft;
    protected DcMotor frontLeft;
    protected DcMotor rearRight;
    protected DcMotor frontRight;

    @Override
    public void init() {
        rearLeft = hardwareMap.dcMotor.get("RearLeft");
        frontLeft = hardwareMap.dcMotor.get("FrontLeft");
        rearRight = hardwareMap.dcMotor.get("RearRight");
        frontRight = hardwareMap.dcMotor.get("FrontRight");

        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        setMDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected void drive(double power) {
        rearLeft.setPower(power);
        frontLeft.setPower(power);
        rearRight.setPower(power);
        frontRight.setPower(power);
    }

    protected void drive(double left, double right) {
        rearLeft.setPower(left);
        frontLeft.setPower(left);
        rearRight.setPower(right);
        frontRight.setPower(right);
    }

    protected void drive(double rearLeft, double frontLeft, double rearRight, double frontRight) {
        this.rearLeft.setPower(rearLeft);
        this.frontLeft.setPower(frontLeft);
        this.rearRight.setPower(rearRight);
        this.frontRight.setPower(frontRight);
    }

    protected void left() {
        drive(1, -1, -1, 1);
    }

    protected void right() {
        drive(-1, 1, 1, -1);
    }

    protected void setMDriveMode(DcMotor.RunMode runMode) {
        rearLeft.setMode(runMode);
        frontLeft.setMode(runMode);
        rearRight.setMode(runMode);
        frontRight.setMode(runMode);
    }
}
