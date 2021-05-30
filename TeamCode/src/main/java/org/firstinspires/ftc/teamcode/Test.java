package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

//@Config
@Disabled
@Autonomous(name="Test", group = "drive")
public class Test extends LinearOpMode {
    public static PIDFCoefficients pid = new PIDFCoefficients(10, 3, 0, 0);
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "LeftShooter");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;


        while (!isStopRequested()) {

            if (gamepad1.a)
                motor.setPower(0.55);
            else if (gamepad1.b)
                motor.setPower(0);

            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);

            telemetry.addData("PID", motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.addData("velocity", motor.getVelocity() / Constants.MotorConstants.REV_HD_HEX.ticks_per_revolution * 60);
            telemetry.update();
        }
    }
}