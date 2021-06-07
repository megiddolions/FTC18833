package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.commandftc.opModes.CommandBasedTeleOp;
import org.firstinspires.ftc.teamcode.Constants.DriveTrainConstants;
import org.firstinspires.ftc.teamcode.lib.kinematics.MecanumDrive;
import org.firstinspires.ftc.teamcode.lib.kinematics.RoadRunnerOdometry;
import org.firstinspires.ftc.teamcode.lib.tragectory.TrajectorySequence;
import org.firstinspires.ftc.teamcode.lib.tragectory.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.lib.tragectory.TrajectorySequenceRunner;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static org.commandftc.RobotUniversal.hardwareMap;
import static org.commandftc.RobotUniversal.opMode;

@Config
public class DriveTrainSubsystem extends com.acmerobotics.roadrunner.drive.MecanumDrive implements MecanumDrive, Subsystem {
    private final DcMotorEx rearLeft;
    private final DcMotorEx rearRight;
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;

    private final BNO055IMU imu;
    private double imu_angle_offset;

    public static PIDCoefficients FORWARD_PID = new PIDCoefficients(2, 0, 0);
    public static PIDCoefficients STRAFE_PID = new PIDCoefficients(7, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(5, 0, 0);

    private final TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(6, Math.toRadians(165), 0.28);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(0.4);

    private final boolean trajectoryControled;

    public DriveTrainSubsystem() {
        super(DriveTrainConstants.kV, DriveTrainConstants.kA, DriveTrainConstants.kStatic, 0.187, 1);
        setLocalizer(new RoadRunnerOdometry(
                new DoubleSupplier[]{
                        this::getLeftOdometryDistance,
                        this::getRightOdometryDistance,
                        this::getHorizontalOdometryDistance},
                new DoubleSupplier[]{
                        this::getLeftOdometryVelocity,
                        this::getRightOdometryVelocity,
                        this::getHorizontalOdometryVelocity
                }));
        rearLeft = (DcMotorEx) hardwareMap.dcMotor.get("RearLeft");
        rearRight = (DcMotorEx)hardwareMap.dcMotor.get( "RearRight");
        frontLeft = (DcMotorEx)hardwareMap.dcMotor.get("FrontLeft");
        frontRight = (DcMotorEx) hardwareMap.dcMotor.get("FrontRight");
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
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        resetAngle();

        TrajectoryFollower follower = new HolonomicPIDVAFollower(FORWARD_PID, STRAFE_PID, HEADING_PID,
                new Pose2d(0.1, 0.1, Math.toRadians(0.5)), 0.5);

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);

        // I couldn't use SubsystemBase so I had to register it here
        CommandScheduler.getInstance().registerSubsystem(this);

        trajectoryControled = !CommandBasedTeleOp.class.isAssignableFrom(opMode.getClass());
    }

    @Override
    public void periodic() {
        updatePoseEstimate();
        if (trajectoryControled) {
            DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
            if (signal != null) setDriveSignal(signal);
        }
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

    public double getLeftOdometryVelocity() {
        return -rearRight.getVelocity() * DriveTrainConstants.kOdometryConstants.meters_per_tick;
    }

    public double getRightOdometryVelocity() {
        return -frontLeft.getVelocity() * DriveTrainConstants.kOdometryConstants.meters_per_tick;
    }

    public double getHorizontalOdometryVelocity() {
        return frontRight.getVelocity() * DriveTrainConstants.kOdometryConstants.meters_per_tick;
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

    public double getHeading() {
        return imu.getAngularOrientation().firstAngle - imu_angle_offset;
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

    public double getRearLeftPower() {
        return rearLeft.getPower();
    }

    public double getRearRightPower() {
        return rearRight.getPower();
    }

    public double getFrontLeftPower() {
        return frontLeft.getPower();
    }

    public double getFrontRightPower() {
        return frontRight.getPower();
    }

    public boolean isBusy() {
//        return frontLeft.isBusy() || frontRight.isBusy() || rearRight.isBusy() || rearLeft.isBusy();
        return trajectorySequenceRunner.isBusy();
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

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(getLeftOdometryDistance(), getRightOdometryDistance(), getHorizontalOdometryDistance());
    }

    @Override
    public void setMotorPowers(double frontLeft, double rearLeft, double rearRight, double frontRight) {
        this.frontLeft.setPower(frontLeft);
        this.rearLeft.setPower(rearLeft);
        this.frontRight.setPower(frontRight);
        this.rearRight.setPower(rearRight);
    }

    @Override
    protected double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, TrajectoryVelocityConstraint VEL_CONSTRAINT) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                Math.toRadians(165), Math.toRadians(165)
        );
    }

    @NotNull
    @Contract("_, _, _ -> new")
    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    @NotNull
    @Contract("_ -> new")
    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void followTrajectoryAsync(@NotNull Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }
}
