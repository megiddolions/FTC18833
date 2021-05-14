package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.commandftc.RobotUniversal;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Disabled
@TeleOp(name="Text Vision")
public class TestVision extends LinearOpMode {
    private VisionSubsystem visionSubsystem;

    @Override
    public void runOpMode() {
        RobotUniversal.telemetry = telemetry;
        RobotUniversal.hardwareMap = hardwareMap;

        VisionSubsystem vision = new VisionSubsystem();
        vision.set_for_autonomous();

        while(opModeIsActive())
            telemetry.update();

        waitForStart();
        telemetry.addData("orange pixels", vision::getOrangePixels);
        telemetry.addData("rings", vision::count_rings);
        while (opModeIsActive()) {
            telemetry.update();
        }
    }
}