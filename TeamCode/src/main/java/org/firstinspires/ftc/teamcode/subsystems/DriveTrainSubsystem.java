package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Constants.DriveTrainConstants;
import org.firstinspires.ftc.teamcode.lib.kinematics.MecanumDrive;
import org.firstinspires.ftc.teamcode.lib.kinematics.RoadRunnerOdometry;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static org.commandftc.RobotUniversal.hardwareMap;

public class DriveTrainSubsystem extends com.acmerobotics.roadrunner.drive.MecanumDrive implements MecanumDrive, Subsystem {
    private final DcMotor rearLeft;
    private final DcMotor rearRight;
    private final DcMotor frontLeft;
    private final DcMotor frontRight;

    private final BNO055IMU imu;
    private double imu_angle_offset;
    public DriveTrainSubsystem() {
        super(2770, 0, 0.187, 0.187, 1);
        setLocalizer(new RoadRunnerOdometry(
                new DoubleSupplier[]{
                        this::getLeftOdometryDistance,
                        this::getRightOdometryDistance,
                        this::getHorizontalOdometryDistance}));
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
        updatePoseEstimate();
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

    public double getLeftOdometryDistance() {
        return getLeftOdometryEncoder() * DriveTrainConstants.kOdometryConstants.meters_per_tick;
    }

    public double getRightOdometryDistance() {
        return getRightOdometryEncoder() * DriveTrainConstants.kOdometryConstants.meters_per_tick;
    }

    public double getHorizontalOdometryDistance() {
        return getHorizontalOdometryEncoder() * DriveTrainConstants.kOdometryConstants.meters_per_tick;
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
    public void holonomicDrive(double x, double y, double a, double power) {
        double[] values = new double[]{+x + y + a, -x + y - a, -x + y + a, +x + y - a};
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

    public Pose2d getPosition() {
        com.acmerobotics.roadrunner.geometry.Pose2d pose = getPoseEstimate();
        return new Pose2d(new Translation2d(pose.getX(), pose.getY()), new Rotation2d(pose.getHeading()));
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(getLeftOdometryDistance(), getRightOdometryDistance(), getHorizontalOdometryDistance());
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        setPower(v, v1, v2, v3);
    }

    @Override
    protected double getRawExternalHeading() {
        return imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES).thirdAngle;
    }

    public Translation2d[] getRobotCorners() {
        Translation2d[] corners = new Translation2d[4];

        corners[0] = getPosition().plus(
                new Transform2d(
                        new Translation2d(.2125, .205).rotateBy(getPosition().getRotation()),
                        new Rotation2d())).getTranslation();

        corners[1] = getPosition().plus(
                new Transform2d(
                        new Translation2d(-.2125, .205).rotateBy(getPosition().getRotation()),
                        new Rotation2d())).getTranslation();

        corners[2] = getPosition().plus(
                new Transform2d(
                        new Translation2d(-.2125, -.205).rotateBy(getPosition().getRotation()),
                        new Rotation2d())).getTranslation();

        corners[3] = getPosition().plus(
                new Transform2d(
                        new Translation2d(.2125, -.205).rotateBy(getPosition().getRotation()),
                        new Rotation2d())).getTranslation();

        return corners;
    }

    public Translation2d[] getIntakeCorners() {
        Translation2d[] corners = new Translation2d[4];

        corners[0] = getPosition().plus(
                new Transform2d(
                        new Translation2d(.26, .1835).rotateBy(getPosition().getRotation()),
                        new Rotation2d())).getTranslation();

        corners[1] = getPosition().plus(
                new Transform2d(
                        new Translation2d(.31, .1835).rotateBy(getPosition().getRotation()),
                        new Rotation2d())).getTranslation();

        corners[2] = getPosition().plus(
                new Transform2d(
                        new Translation2d(.31, -.1835).rotateBy(getPosition().getRotation()),
                        new Rotation2d())).getTranslation();

        corners[3] = getPosition().plus(
                new Transform2d(
                        new Translation2d(.26, -.1835).rotateBy(getPosition().getRotation()),
                        new Rotation2d())).getTranslation();

        return corners;
    }

    private void draw_robot_base(Canvas canvas) {
        Translation2d[] corners = getRobotCorners();

        double[] raw_x = new double[]{corners[0].getX(), corners[1].getX(), corners[2].getX(), corners[3].getX()};
        double[] raw_y = new double[]{corners[0].getY(), corners[1].getY(), corners[2].getY(), corners[3].getY()};

        for (int i = 0; i < 4; i++) {
            raw_x[i] -= 1.8288;
            raw_y[i] -= 1.8288;

            raw_x[i] *= 3.6576 * 12;
            raw_y[i] *= 3.6576 * 12;
        }


        canvas.strokePolygon(raw_x, raw_y);

        canvas.setFill("#ff0000");

        canvas.fillCircle((getPosition().getX()  - 1.8288) * 3.6576 * 12,
                (getPosition().getY() - 1.8288) * 3.6576 * 12,1);
    }

    private void draw_robot_intake(Canvas canvas) {
        Translation2d[] corners = getIntakeCorners();

        double[] raw_x = new double[]{corners[0].getX(), corners[1].getX(), corners[2].getX(), corners[3].getX()};
        double[] raw_y = new double[]{corners[0].getY(), corners[1].getY(), corners[2].getY(), corners[3].getY()};

        for (int i = 0; i < 4; i++) {
            raw_x[i] -= 1.8288;
            raw_y[i] -= 1.8288;

            raw_x[i] *= 3.6576 * 12;
            raw_y[i] *= 3.6576 * 12;
        }


        canvas.strokePolygon(raw_x, raw_y);
    }

    private void draw_robot(Canvas canvas) {
        canvas.setStroke("#f10fc7");
        draw_robot_base(canvas);
        draw_robot_intake(canvas);
    }

    public TelemetryPacket draw_robot(@NotNull TelemetryPacket packet) {

        Canvas canvas = packet.fieldOverlay();

        draw_robot(canvas);

        return packet;
    }
}
