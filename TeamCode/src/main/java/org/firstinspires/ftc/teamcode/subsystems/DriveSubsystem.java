package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    private final DcMotorEx rearLeft;
    private final DcMotorEx rearRight;
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;

    public DriveSubsystem(HardwareMap hardwareMap) {
        rearLeft = hardwareMap.get(DcMotorEx.class, "RearLeft");
        rearRight = hardwareMap.get(DcMotorEx.class, "RearRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");

        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setMode(DcMotor.RunMode mode) {
        rearLeft.setMode(mode);
        rearRight.setMode(mode);
        rearLeft.setMode(mode);
        rearLeft.setMode(mode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        rearLeft.setZeroPowerBehavior(behavior);
        rearRight.setZeroPowerBehavior(behavior);
        rearLeft.setZeroPowerBehavior(behavior);
        rearLeft.setZeroPowerBehavior(behavior);
    }

    public void stop() {
        rearLeft.setPower(0);
        rearRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
    }

    public void setPower(double left, double right) {
        rearLeft.setPower(left);
        rearRight.setPower(right);
        frontLeft.setPower(left);
        frontRight.setPower(right);
    }

    public void setPower(double RL, double FL, double RR, double FR) {
        rearLeft.setPower(RL);
        rearRight.setPower(FL);
        frontLeft.setPower(RR);
        frontRight.setPower(FR);
    }

    public void driveLeft(double speed) {
        rearLeft.setPower(speed);
        rearRight.setPower(-speed);
        frontLeft.setPower(-speed);
        frontRight.setPower(speed);
    }

    public void driveRight(double speed) {
        rearLeft.setPower(-speed);
        rearRight.setPower(speed);
        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
    }

    public int getRearLeftEncoder() {
        return rearLeft.getCurrentPosition();
    }

    public int getRearRightEncoder() {
        return rearRight.getCurrentPosition();
    }

    public int getFrontLeftEncoder() {
        return frontLeft.getCurrentPosition();
    }

    public int getFrontRightEncoder() {
        return frontRight.getCurrentPosition();
    }

    public void differentialDrive(double forward, double tankPivot, double strafing) {
        frontLeft.setPower(forward + tankPivot - strafing);
        rearLeft.setPower(forward + tankPivot + strafing);
        frontRight.setPower(forward - tankPivot + strafing);
        rearRight.setPower(forward - tankPivot - strafing);
    }
}
