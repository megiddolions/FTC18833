package org.firstinspires.ftc.teamcode.vison.pipelines.align;

import org.openftc.easyopencv.OpenCvPipeline;

public abstract class AlignPipeLine extends OpenCvPipeline {
    public abstract double getError();
}
