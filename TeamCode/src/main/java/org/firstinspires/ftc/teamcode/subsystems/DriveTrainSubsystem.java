package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.DriveTrainConstants;
import org.firstinspires.ftc.teamcode.lib.kinematics.MecanumDrive;
import org.jetbrains.annotations.NotNull;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.commandftc.RobotUniversal.hardwareMap;

public class DriveTrainSubsystem extends SubsystemBase implements MecanumDrive {
    private final DcMotor rearLeft;
    private final DcMotor rearRight;
    private final DcMotor frontLeft;
    private final DcMotor frontRight;

    private final BNO055IMU imu;
    private double imu_angle_offset;

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
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu.initialize(parameters);

        resetAngle();
    }

    @Override
    public void periodic() {
        update_position();
    }

    public int getLeftOdometryEncoder() {
        return -rearRight.getCurrentPosition();
    }

    public int getRightOdometryEncoder() {
        return -frontLeft.getCurrentPosition();
    }

    public int getHorizontalOdometryEncoder() {
        return frontRight.getCurrentPosition();
    }

    public double getLeftOdometryWheel() {
        return getLeftOdometryEncoder() / Constants.MotorConstants.REV_THROUGH_BORE_ENCODER.ticks_per_revolution * 60 * 2 * Math.PI;
    }

    public double getRightOdometryWheel() {
        return getRightOdometryEncoder() / Constants.MotorConstants.REV_THROUGH_BORE_ENCODER.ticks_per_revolution * 60 * 2 * Math.PI;
    }

    public double getHorizontalOdometryWheel() {
        return getHorizontalOdometryEncoder() / Constants.MotorConstants.REV_THROUGH_BORE_ENCODER.ticks_per_revolution * 60 * 2 * Math.PI;
    }

    public boolean isGyroCalibrated() {
        return imu.isGyroCalibrated();
    }

    @Deprecated
    public void set_for_autonomous() {
        rearLeft.setTargetPosition(rearLeft.getCurrentPosition());
        rearRight.setTargetPosition(rearRight.getCurrentPosition());
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition());
        frontRight.setTargetPosition(frontRight.getCurrentPosition());
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(1);
    }

    @Deprecated
    public void set_for_drive() {
        setPower(0);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Deprecated
    public void reset_encoders() {
        DcMotor.RunMode mode = frontLeft.getMode();
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(mode);
    }

    public Rotation2d getHeading() {
        return new Rotation2d((imu.getAngularOrientation().firstAngle - imu_angle_offset)
                / 180 * Math.PI); // Convert to radians
    }

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
        rearLeft.setPower(power);
        rearRight.setPower(power);
        frontLeft.setPower(power);
        frontRight.setPower(power);
    }

    @Override
    public void tankDrive(double left, double right) {
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

    public void setPower(@NotNull double[] values) {
        rearLeft.setPower(values[0]);
        rearRight.setPower(values[1]);
        frontLeft.setPower(values[2]);
        frontRight.setPower(values[3]);
    }

    @Override
    public void driveLeft(double speed) {
        rearLeft.setPower(speed);
        rearRight.setPower(-speed);
        frontLeft.setPower(-speed);
        frontRight.setPower(speed);
    }

    @Override
    public void driveRight(double speed) {
        rearLeft.setPower(-speed);
        rearRight.setPower(speed);
        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
    }

    @Override
    public void driveDirection(double x, double y, double a, double power) {
        double[] values = new double[]{x + y + a, -x + y - a, -x + y + a, +x + y - a};
        // Find max power
        double max = Math.max(Math.max(Math.abs(values[0]),Math.abs(values[1])),Math.max(Math.abs(values[2]),Math.abs(values[3])));
        // Keep all motor values in range
        if (max > power) {
            values[0] /= max * power;
            values[1] /= max * power;
            values[2] /= max * power;
            values[3] /= max * power;
        }

        setPower(values);
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
        frontRight.setTargetPosition((int)(frontRight.getCurrentPosition() + DriveTrainConstants.mm_to_ticks.apply(2.5*mm)));
    }

    public void driveDiagonalRight(double mm) {
        rearRight.setTargetPosition((int)(rearRight.getCurrentPosition() + DriveTrainConstants.mm_to_ticks.apply(2.5*mm)));
        frontLeft.setTargetPosition((int)(frontLeft.getCurrentPosition() + DriveTrainConstants.mm_to_ticks.apply(2.5*mm)));
    }


    /// Odometry stuff ///
    private Pose2d current_position = new Pose2d();

    public Pose2d getPosition() {
        return current_position;
    }

    public Pose2d getRobotCenterPosition() {
        return current_position.plus(new Transform2d(
                new Translation2d(0, 0.175).rotateBy(current_position.getRotation().unaryMinus()),
                new Rotation2d()));
    }

    public void setPosition(Pose2d position) {
        current_position = position;
    }

    private int last_left = 0;
    private int last_right = 0;
    private int last_horizontal = 0;

    private void update_position() {
        int left = getLeftOdometryEncoder();
        int right = getRightOdometryEncoder();
        int horizontal = getHorizontalOdometryEncoder();

        int delta_left = left - last_left;
        int delta_right = right - last_right;
        int delta_horizontal = horizontal - last_horizontal;

        double delta_angle = (delta_right - delta_left)
                / DriveTrainConstants.kOdometryConstants.getVerticalWheelsDistance()
                * DriveTrainConstants.kOdometryConstants.meters_per_tick;

        // The arc length of movement forward/backward
        double forward_movement = (delta_left  + delta_right) / 2d * DriveTrainConstants.kOdometryConstants.meters_per_tick;

        // The arc length of movement left/right
        double left_movement = (delta_horizontal) * DriveTrainConstants.kOdometryConstants.meters_per_tick
                + delta_angle * DriveTrainConstants.kOdometryConstants.horizontalWheel
                .getDistance(new Translation2d(0, -0.175));

        // Calculate the new angle of the robot using the difference between the left and right encoder
        current_position = current_position.plus(
                new Transform2d(
                        new Translation2d(
                                left_movement,
                                forward_movement
                        ).rotateBy(current_position.getRotation().plus(new Rotation2d(delta_angle))),
                        // Add change in angle to current angle
                        new Rotation2d(delta_angle)
                )
        );

        last_left = left;
        last_right = right;
        last_horizontal = horizontal;
    }
}
