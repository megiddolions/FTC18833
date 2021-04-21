package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    private final DcMotorEx rearLeft;
    private final DcMotorEx rearRight;
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;

    private double angler_drive_speed_modifier = 1;
    private double vertical_drive_speed_modifier = 1;
    private double horizontal_drive_speed_modifier = 1;

    public DriveSubsystem() {
        rearLeft = Robot.OpMode().hardwareMap.get(DcMotorEx.class, "RearLeft");
        rearRight = Robot.OpMode().hardwareMap.get(DcMotorEx.class, "RearRight");
        frontLeft = Robot.OpMode().hardwareMap.get(DcMotorEx.class, "FrontLeft");
        frontRight = Robot.OpMode().hardwareMap.get(DcMotorEx.class, "FrontRight");

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

    public void setAnglerSpeed(double modifier) {
        angler_drive_speed_modifier = modifier;
    }

    public void setVerticalSpeed(double modifier) {
        vertical_drive_speed_modifier = modifier;
    }

    public void setHorizontalSpeed(double modifier) {
        horizontal_drive_speed_modifier = modifier;
    }

    public void differentialDrive(double y, double x, double spin) {
        double raw_spin = spin * angler_drive_speed_modifier;
        double raw_x = x * horizontal_drive_speed_modifier;
        double raw_y = y * vertical_drive_speed_modifier;

        frontLeft.setPower(-raw_spin+raw_y+raw_x);
        rearLeft.setPower(-raw_spin+raw_y-raw_x);
        frontRight.setPower(raw_spin+raw_y-raw_x);
        rearRight.setPower(raw_spin+raw_y+raw_x);
    }
}
