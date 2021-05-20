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
import org.firstinspires.ftc.teamcode.lib.OdometrySmaples.Odometry1;
import org.firstinspires.ftc.teamcode.lib.OdometrySmaples.Odometry2;
import org.firstinspires.ftc.teamcode.lib.OdometrySmaples.Odometry3;
import org.firstinspires.ftc.teamcode.lib.OdometrySmaples.Odometry4;
import org.firstinspires.ftc.teamcode.lib.OdometrySmaples.OdometryInterface;
import org.firstinspires.ftc.teamcode.lib.kinematics.OdometryConstants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobellSubsystem;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import static org.commandftc.RobotUniversal.hardwareMap;

@TeleOp(name="Odometry")
public class TestDrive extends OpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor rearLeft;
    private DcMotor rearRight;

    OdometryInterface odometry;

    @Override
    public void init() {
        rearLeft = hardwareMap.dcMotor.get("RearLeft");
        rearRight = hardwareMap.dcMotor.get( "RearRight");
        frontLeft = hardwareMap.dcMotor.get("FrontLeft");
        frontRight = hardwareMap.dcMotor.get("FrontRight");

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        odometry = new Odometry4();
    }

    @Override
    public void loop() {
        frontLeft.setPower(-gamepad1.left_stick_y);
        rearLeft.setPower(-gamepad1.left_stick_y);
        frontRight.setPower(-gamepad1.right_stick_y);
        rearRight.setPower(-gamepad1.right_stick_y);

        odometry.update(getLeftTicks(),
                getRightTicks(),
                -getCenterTicks());

        telemetry.addData("x", odometry.getX());
        telemetry.addData("y", odometry.getY());
        telemetry.addData("a", odometry.getA());
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
    public int getLeftTicks() {
        return -rearRight.getCurrentPosition();
    }
    public int getRightTicks() {
        return -frontLeft.getCurrentPosition();
    }
    public int getCenterTicks() {
        return frontRight.getCurrentPosition();
    }


}
