package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.commandftc.RobotUniversal;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobellSubsystem;

@Disabled
@TeleOp(name = "TestServo")
public class TestServo extends OpMode {
    WobellSubsystem wobell;

    @Override
    public void init() {
        RobotUniversal.hardwareMap = hardwareMap;
        RobotUniversal.opMode = this;
        RobotUniversal.telemetry = telemetry;

        wobell = new WobellSubsystem();
    }

    @Override
    public void loop() {
        wobell.setLift(-gamepad1.left_stick_y);
        if (gamepad1.dpad_left)
            wobell.open();
        else if (gamepad1.dpad_right)
            wobell.close();
    }
}
