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

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import static org.commandftc.RobotUniversal.hardwareMap;

@TeleOp(name="Odometry")
public class TestDrive extends OpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor rearLeft;
    private DcMotor rearRight;

    // Drive train moves according to these. Update them, it moves
    private LocalCoordinateSystem lcs = new LocalCoordinateSystem();

    // Right - 2
    // Left - 1
    // Horizontal - 3

    @Override
    public void init() {
        rearLeft = hardwareMap.dcMotor.get("RearLeft");
        rearRight = hardwareMap.dcMotor.get( "RearRight");
        frontLeft = hardwareMap.dcMotor.get("FrontLeft");
        frontRight = hardwareMap.dcMotor.get("FrontRight");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {
        frontLeft.setPower(-gamepad1.left_stick_y);
        rearLeft.setPower(-gamepad1.left_stick_y);
        frontRight.setPower(-gamepad1.right_stick_y);
        rearRight.setPower(-gamepad1.right_stick_y);

        lcs.update(
                getLeftTicks(),
                getRightTicks(),
                getCenterTicks()
        );

        telemetry.addData("x", lcs.x);
        telemetry.addData("y", lcs.y);
        telemetry.addData("a", lcs.a);
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

    private static class LocalCoordinateSystem {
        public double x = 0;    // The approximated x position of the robot relative to where it started
        public double y = 0;    // The approximated y position of the robot relative to where it started
        public double a = 0;    // The approximated heading of the robot relative to its initial heading

        public double prev_le;
        public double prev_re;
        public double prev_ce;


        private final double ENCODER_CPR          = Constants.DriveTrainConstants.kOdometryConstants.ticks_per_revolution;             // Counts per full rotation of an encoder
        private final double ROBOT_DIAMETER       = Constants.DriveTrainConstants.kOdometryConstants.getVerticalWheelsDistance();      //15.7075609922;    //15.74735 //15.53           // Distance between the left and right encoder (diameter) in inches
        private final double CENTER_WHEEL_OFFSET  = Constants.DriveTrainConstants.kOdometryConstants.getHorizontalWheelOffset();      //7.725136416;      //7.719 //7.375 Distance of the center encoder to the line made between the left and right encoders (radius) in inches

        private final double WHEEL_DIAMETER     = Constants.DriveTrainConstants.kOdometryConstants.wheel_diameter;

        private final double METER_PER_COUNT   = WHEEL_DIAMETER * Math.PI / ENCODER_CPR;

        public void update(double le, double re, double ce) {

            // Calculate encoder deltas
            double ld = le - prev_le;
            double rd = re - prev_re;
            double cd = ce - prev_ce;

            // Calculate phi, or the delta of our angle
            double ph = (rd * METER_PER_COUNT - ld * METER_PER_COUNT) / ROBOT_DIAMETER;

            // The arc length of movement forward/backward
            double dc = (rd * METER_PER_COUNT + ld * METER_PER_COUNT) / 2;

            // The arc length of movement left/right
            double sc = (cd * METER_PER_COUNT) + (ph * CENTER_WHEEL_OFFSET);

            // Calculate the new angle of the robot using the difference between the left and right encoder
            a = (re * METER_PER_COUNT - le * METER_PER_COUNT) / ROBOT_DIAMETER;

            // Calculate the new position of the robot by adding the arc vector to the absolute pos
            double sinph = Math.sin(ph);
            double cosph = Math.cos(ph);

            double s;
            double c;

            // If the arc turn is small enough, do this instead to avoid a div by zero error
            if(Math.abs(ph) < 1E-9) {
                s = 1.0 - 1.0 / 6.0 * ph * ph;
                c = 0.5 * ph;
            } else {
                s = sinph / ph;
                c = (1.0 - cosph) / ph;
            }

            // Find our x and y translations relative to the origin pose (0,0,0)
            double rel_x = sc * s - dc * c;
            double rel_y = sc * c - dc * s;

            // Transform those x and y translations to the actual rotation of our robot, and translate our robots positions to the new spot
            x -= rel_x * Math.cos(a) - rel_y * Math.sin(a);
            y -= rel_x * Math.sin(a) + rel_y * Math.cos(a);

        /* OLD BAD BAD BAD CODE THAT DOESN'T REALLY WORK AT ALL REALLY
        y += (dc * Math.cos(a + (ph / 2))) - (sc * Math.sin(a + (ph / 2)));
        x -= (dc * Math.sin(a + (ph / 2))) + (sc * Math.cos(a + (ph / 2)));
        */

            // Used to calculate deltas for next loop
            prev_le = le;
            prev_re = re;
            prev_ce = ce;

        }
    }
}
