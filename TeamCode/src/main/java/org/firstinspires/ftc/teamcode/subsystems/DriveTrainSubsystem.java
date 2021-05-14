package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constants.DriveTrainConstants;
import org.firstinspires.ftc.teamcode.lib.kinematics.Odometry;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.commandftc.RobotUniversal.hardwareMap;

public class DriveTrainSubsystem extends SubsystemBase {
    private final DcMotor rearLeft;
    private final DcMotor rearRight;
    private final DcMotor frontLeft;
    private final DcMotor frontRight;

    public double drive_speed = 0.8;

//    private double angler_drive_speed_modifier = 1;
//    private double vertical_drive_speed_modifier = 1;
//    private double horizontal_drive_speed_modifier = 1;

    private final BNO055IMU imu;
    private double imu_angle_offset;

//    public MecanumDriveOdometry odometry;
//    public Odometry odometry;

    public DriveTrainSubsystem() {
        rearLeft = hardwareMap.dcMotor.get("RearLeft");
        rearRight = hardwareMap.dcMotor.get( "RearRight");
        frontLeft = hardwareMap.dcMotor.get("FrontLeft");
        frontRight = hardwareMap.dcMotor.get("FrontRight");
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
        rearLeft.setTargetPosition(0);
        rearRight.setTargetPosition(0);
        frontLeft.setTargetPosition(0);
        frontRight.setTargetPosition(0);
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

//        odometry = new MecanumDriveOdometry(DriveTrainConstants.kinematics, getHeading(), new Pose2d(0, 0, new Rotation2d()));
//        odometry = new Odometry(DriveTrainConstants.kOdometryConstants,
//                this::getLeftOdometryWheel,
//                this::getRightOdometryWheel,
//                this::getHorizontalOdometryWheel);
    }

    @Override
    public void periodic() {
//        odometry.update();
    }

    public double getLeftOdometryWheel() {
        return rearRight.getCurrentPosition() / 8192.0 * 60 * 2 * Math.PI;
    }

    public double getRightOdometryWheel() {
        return frontLeft.getCurrentPosition() / 8192.0 * 60 * 2 * Math.PI;
    }

    public double getHorizontalOdometryWheel() {
        return frontRight.getCurrentPosition() / 8192.0 * 60 * 2 * Math.PI;
    }

    public boolean isGyroCalibrated() {
        return imu.isGyroCalibrated();
    }
//    @Deprecated
    public void set_for_autonomous() {
        rearLeft.setTargetPosition(rearLeft.getCurrentPosition());
        rearRight.setTargetPosition(rearRight.getCurrentPosition());
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition());
        frontRight.setTargetPosition(frontRight.getCurrentPosition());
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive_speed = 1;
        setPower(1);
    }

    public void set_for_drive() {
        drive_speed = 1;
        setPower(0);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void reset_encoders() {
        DcMotor.RunMode mode = frontLeft.getMode();
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(mode);
    }

//    public MecanumDriveWheelSpeeds getWheelSpeeds() {
//        return new MecanumDriveWheelSpeeds(
//                frontLeft_getRate(), frontRight_getRate(),
//                rearLeft_getRate(), rearRight_getRate());
//    }

    public Rotation2d getHeading() {
        return new Rotation2d((imu.getAngularOrientation().firstAngle - imu_angle_offset)
                / 180 * Math.PI); // Convert to radians
    }

//    public double frontLeft_getRate() {
//        return frontLeft.getVelocity() / DriveTrainConstants.ticks_per_revolution;
//    }
//
//    public double frontRight_getRate() {
//        return frontRight.getVelocity() / DriveTrainConstants.ticks_per_revolution;
//    }
//
//    public double rearLeft_getRate() {
//        return rearLeft.getVelocity() / DriveTrainConstants.ticks_per_revolution;
//    }
//
//    public double rearRight_getRate() {
//        return rearRight.getVelocity() / DriveTrainConstants.ticks_per_revolution;
//    }

    public void resetAngle() {
        imu_angle_offset = imu.getAngularOrientation().firstAngle;
    }

    public void setMode(DcMotor.RunMode mode) {
        rearLeft.setMode(mode);
        rearRight.setMode(mode);
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        rearLeft.setZeroPowerBehavior(behavior);
        rearRight.setZeroPowerBehavior(behavior);
        frontLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
    }

    public void stop() {
        rearLeft.setPower(0);
        rearRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
    }

    public void setPower(double power) {
        rearLeft.setPower(power * drive_speed);
        rearRight.setPower(power * drive_speed);
        frontLeft.setPower(power * drive_speed);
        frontRight.setPower(power * drive_speed);
    }

    public void setPower(double left, double right) {
        rearLeft.setPower(left * drive_speed);
        rearRight.setPower(right * drive_speed);
        frontLeft.setPower(left * drive_speed);
        frontRight.setPower(right * drive_speed);
    }

    public void setPower(double RL, double FL, double RR, double FR) {
        rearLeft.setPower(RL * drive_speed);
        rearRight.setPower(FL * drive_speed);
        frontLeft.setPower(RR * drive_speed);
        frontRight.setPower(FR * drive_speed);
    }

    public void driveLeft(double speed) {
        rearLeft.setPower(speed * drive_speed);
        rearRight.setPower(-speed * drive_speed);
        frontLeft.setPower(-speed * drive_speed);
        frontRight.setPower(speed * drive_speed);
    }

    public void driveRight(double speed) {
        rearLeft.setPower(-speed * drive_speed);
        rearRight.setPower(speed * drive_speed);
        frontLeft.setPower(speed * drive_speed);
        frontRight.setPower(-speed * drive_speed);
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

    public int getRearLeftTarget() {
        return rearLeft.getTargetPosition();
    }

    public int getRearRightTarget() {
        return rearRight.getTargetPosition();
    }

    public int getFrontLeftTarget() {
        return frontLeft.getTargetPosition();
    }

    public int getFrontRightTarget() {
        return frontRight.getTargetPosition();
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

    public boolean isBusy() {
        return frontLeft.isBusy() || frontRight.isBusy() || rearRight.isBusy() || rearLeft.isBusy();
    }

    public void driveForwardDistance(double mm) {
        rearLeft.setTargetPosition((int)(rearLeft.getCurrentPosition() + DriveTrainConstants.mm_to_ticks.apply(mm)));
        rearRight.setTargetPosition((int)(rearRight.getCurrentPosition() + DriveTrainConstants.mm_to_ticks.apply(mm)));
        frontLeft.setTargetPosition((int)(frontLeft.getCurrentPosition() + DriveTrainConstants.mm_to_ticks.apply(mm)));
        frontRight.setTargetPosition((int)(frontRight.getCurrentPosition() + DriveTrainConstants.mm_to_ticks.apply(mm)));
    }

    public void spinLeftDistance(double spin) {
        rearLeft.setTargetPosition((int)(rearLeft.getCurrentPosition() - DriveTrainConstants.mm_to_ticks.apply(spin)));
        rearRight.setTargetPosition((int)(rearRight.getCurrentPosition() + DriveTrainConstants.mm_to_ticks.apply(spin)));
        frontLeft.setTargetPosition((int)(frontLeft.getCurrentPosition() - DriveTrainConstants.mm_to_ticks.apply(spin)));
        frontRight.setTargetPosition((int)(frontRight.getCurrentPosition() + DriveTrainConstants.mm_to_ticks.apply(spin)));
    }

    public void driveLeftDistance(double mm) {
        rearLeft.setTargetPosition((int)(rearLeft.getCurrentPosition() - DriveTrainConstants.mm_to_ticks.apply(7*mm)));
        rearRight.setTargetPosition((int)(rearRight.getCurrentPosition() + DriveTrainConstants.mm_to_ticks.apply(7*mm)));
        frontLeft.setTargetPosition((int)(frontLeft.getCurrentPosition() + DriveTrainConstants.mm_to_ticks.apply(7*mm)));
        frontRight.setTargetPosition((int)(frontRight.getCurrentPosition() - DriveTrainConstants.mm_to_ticks.apply(7*mm)));
    }

    public void driveDiagonalLeft(double mm) {
        rearLeft.setTargetPosition((int)(rearLeft.getCurrentPosition() + DriveTrainConstants.mm_to_ticks.apply(2.5*mm)));
//        rearRight.setTargetPosition((int)(rearRight.getCurrentPosition() + DriveTrainConstants.mm_to_ticks.apply(mm)));
//        frontLeft.setTargetPosition((int)(frontLeft.getCurrentPosition() + DriveTrainConstants.mm_to_ticks.apply(mm)));
        frontRight.setTargetPosition((int)(frontRight.getCurrentPosition() + DriveTrainConstants.mm_to_ticks.apply(2.5*mm)));
    }

    public void driveDiagonalRight(double mm) {
//        rearLeft.setTargetPosition((int)(rearLeft.getCurrentPosition() + DriveTrainConstants.mm_to_ticks.apply(mm)));
        rearRight.setTargetPosition((int)(rearRight.getCurrentPosition() + DriveTrainConstants.mm_to_ticks.apply(2.5*mm)));
        frontLeft.setTargetPosition((int)(frontLeft.getCurrentPosition() + DriveTrainConstants.mm_to_ticks.apply(2.5*mm)));
//        frontRight.setTargetPosition((int)(frontRight.getCurrentPosition() + DriveTrainConstants.mm_to_ticks.apply(mm)));
    }
}
