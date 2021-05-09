package org.firstinspires.ftc.teamcode.vison.pipelines.align;

import org.opencv.core.Mat;

public class NonePipeLine extends AlignPipeLine {
    @Override
    public double getError() {
        return 0;
    }

    @Override
    public Mat processFrame(Mat input) {
        return input;
    }
}
