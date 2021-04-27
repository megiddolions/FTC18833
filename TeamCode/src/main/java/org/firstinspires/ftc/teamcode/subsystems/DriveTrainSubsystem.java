package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.commandftc.Subsystem;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveWheelSpeeds;

import org.firstinspires.ftc.teamcode.Constants.DriveTrainConstants;

import static org.commandftc.RobotUniversal.*;

public class DriveTrainSubsystem extends Subsystem {
    private final DcMotorEx rearLeft;
    private final DcMotorEx rearRight;
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;

//    private double angler_drive_speed_modifier = 1;
//    private double vertical_drive_speed_modifier = 1;
//    private double horizontal_drive_speed_modifier = 1;

    private final BNO055IMU imu;
    private double imu_angle_offset;

    public MecanumDriveKinematics kinematics;
    public MecanumDriveOdometry odometry;

    public DriveTrainSubsystem() {
        rearLeft = hardwareMap.get(DcMotorEx.class, "RearLeft");
        rearRight = hardwareMap.get(DcMotorEx.class, "RearRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu.initialize(parameters);

        resetAngle();

        kinematics = new MecanumDriveKinematics(
                new Translation2d(0.28, 0.34),
                new Translation2d(0.28, -0.34),
                new Translation2d(-0.28, 0.34),
                new Translation2d(-0.28, -0.34)
        );
//        odometry = new MecanumDriveOdometry(kinematics, getHeading(), new Pose2d(0, 0, new Rotation2d()));
    }

    @Override
    public void periodic() {

//        odometry.update(getHeading(), getWheelSpeeds());
    }

    public MecanumDriveWheelSpeeds getWheelSpeeds() {
        return new MecanumDriveWheelSpeeds(
                frontLeft_getRate(), frontRight_getRate(),
                rearLeft_getRate(), rearRight_getRate());
    }

    public Rotation2d getHeading() {
        return new Rotation2d((imu.getAngularOrientation().firstAngle - imu_angle_offset)
                / 180 * Math.PI); // Convert to radians
    }

    public double frontLeft_getRate() {
        return frontLeft.getVelocity() / DriveTrainConstants.ticks_per_revolution;
    }

    public double frontRight_getRate() {
        return frontRight.getVelocity() / DriveTrainConstants.ticks_per_revolution;
    }

    public double rearLeft_getRate() {
        return rearLeft.getVelocity() / DriveTrainConstants.ticks_per_revolution;
    }

    public double rearRight_getRate() {
        return rearRight.getVelocity() / DriveTrainConstants.ticks_per_revolution;
    }

    public void resetAngle() {
        imu_angle_offset = imu.getAngularOrientation().firstAngle;
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

//    public void setAnglerSpeed(double modifier) {
//        angler_drive_speed_modifier = modifier;
//    }
//
//    public void setVerticalSpeed(double modifier) {
//        vertical_drive_speed_modifier = modifier;
//    }
//
//    public void setHorizontalSpeed(double modifier) {
//        horizontal_drive_speed_modifier = modifier;
//    }
//
//    public void differentialDrive(double y, double x, double spin) {
//        double raw_spin = spin * angler_drive_speed_modifier;
//
//        double vector_abs = Math.sqrt(x*x + y*y);
//        double angle = (x < 0 ? Math.PI : 0) + (x != 0 ? Math.atan(y/x) : Math.PI / 2) - getHeading().getRadians() * 0;
//
//        double raw_x = vector_abs * Math.cos(angle) * horizontal_drive_speed_modifier;
//        double raw_y = vector_abs * Math.sin(angle) * vertical_drive_speed_modifier;
//
//        frontLeft.setPower(-raw_spin+raw_y+raw_x);
//        rearLeft.setPower(-raw_spin+raw_y-raw_x);
//        frontRight.setPower(raw_spin+raw_y-raw_x);
//        rearRight.setPower(raw_spin+raw_y+raw_x);
//    }
}
