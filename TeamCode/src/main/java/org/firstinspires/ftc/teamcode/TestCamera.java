package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vison.SaveVideoPipeLine;
import org.opencv.videoio.VideoWriter;

@TeleOp(name = "Test Camera")
public class TestCamera extends Default {
    VideoWriter videoWriter;

    @Override
    public void init() {
        super.init();
        vision.camera.setPipeline(new SaveVideoPipeLine());
    }

    @Override
    public void loop() {

    }
}
