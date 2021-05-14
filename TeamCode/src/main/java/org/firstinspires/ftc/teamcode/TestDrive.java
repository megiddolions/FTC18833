package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.commandftc.opModes.CommandBasedTeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.commands.Util.LoopTimeCommand;
import org.firstinspires.ftc.teamcode.commands.Wobell.OpenWobellCommand;
import org.firstinspires.ftc.teamcode.commands.Wobell.WobellTargetPositionCommand;
import org.firstinspires.ftc.teamcode.lib.kinematics.OdometryConstants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobellSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import static org.commandftc.RobotUniversal.hardwareMap;

@TeleOp(name="Odometry")
public class TestDrive extends OpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor rearLeft;
    private DcMotor rearRight;
    private BNO055IMU imu;

    private int leftEncoderPos = 0;
    private int centerEncoderPos = 0;
    private int rightEncoderPos = 0;

    double x;
    double y;
    double theta;

    // Right - 2
    // Left - 1
    // Horizontal - 3

    @Override
    public void init() {
        rearLeft = hardwareMap.dcMotor.get("RearLeft");
        rearRight = hardwareMap.dcMotor.get( "RearRight");
        frontLeft = hardwareMap.dcMotor.get("FrontLeft");
        frontRight = hardwareMap.dcMotor.get("FrontRight");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        x = 0;
        y = 0;
        theta = 0;
    }

    @Override
    public void loop() {
        frontLeft.setPower(-gamepad1.left_stick_y);
        rearLeft.setPower(-gamepad1.left_stick_y);
        frontRight.setPower(-gamepad1.right_stick_y);
        rearRight.setPower(-gamepad1.right_stick_y);

        updatePosition();

        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("angle", angle());
        telemetry.addData("Left", getLeftTicks());
        telemetry.addData("Right", getRightTicks());
        telemetry.addData("Center", getCenterTicks());
        telemetry.update();
    }

    public void setMode(DcMotor.RunMode mode) {
        rearLeft.setMode(mode);
        rearRight.setMode(mode);
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
    }
    public void resetTicks() {
        resetLeftTicks();
        resetCenterTicks();
        resetRightTicks();
    }
    public void resetLeftTicks() {
        leftEncoderPos = -rearRight.getCurrentPosition();
    }
    public int getLeftTicks() {
        return -rearRight.getCurrentPosition() - leftEncoderPos;
    }
    public void resetRightTicks() {
        rightEncoderPos = -frontLeft.getCurrentPosition();
    }
    public int getRightTicks() {
        return -frontLeft.getCurrentPosition() - rightEncoderPos;
    }
    public void resetCenterTicks() {
        centerEncoderPos = frontRight.getCurrentPosition();
    }
    public int getCenterTicks() {
        return frontRight.getCurrentPosition() - centerEncoderPos;
    }
    public void updatePosition() {
        double deltaLeftDistance = (getLeftTicks() / Constants.MotorConstants.REV_THROUGH_BORE_ENCODER.ticks_per_revolution) * 2.0 * Math.PI * 0.06;
        double deltaRightDistance = (getRightTicks() / Constants.MotorConstants.REV_THROUGH_BORE_ENCODER.ticks_per_revolution) * 2.0 * Math.PI * 0.06;
        double deltaCenterDistance = (getCenterTicks() / Constants.MotorConstants.REV_THROUGH_BORE_ENCODER.ticks_per_revolution) * 2.0 * Math.PI * 0.06;
        x  += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.cos(theta);
        y  += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.sin(theta);
        theta  += (deltaLeftDistance - deltaRightDistance) / Constants.DriveTrainConstants.kOdometryConstants.getVerticalWheelsDistance();
        resetTicks();
    }
    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public double getTheta() {
        return theta;
    }
    public void setX(double _x) {
        x = _x;
    }
    public void setY(double _y) {
        y = _y;
    }
    public void setTheta(double _theta) {
        theta = _theta;
    }
    public double angle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }
}
