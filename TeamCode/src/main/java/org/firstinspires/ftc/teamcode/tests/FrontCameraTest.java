package org.firstinspires.ftc.teamcode.tests;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.commandftc.opModes.CommandBasedTeleOp;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.vison.pipelines.ViewPipeLine;

@TeleOp(name="FrontCameraTest", group = "tests")
public class FrontCameraTest extends CommandBasedTeleOp {
    protected VisionSubsystem vision;
    @Override
    public void assign() {
        vision = new VisionSubsystem();
        vision.frontCamera.setPipeline(new ViewPipeLine());
    }
}
