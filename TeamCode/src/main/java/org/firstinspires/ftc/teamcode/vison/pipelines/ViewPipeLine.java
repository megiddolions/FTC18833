package org.firstinspires.ftc.teamcode.vison.pipelines;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;

import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.camera_height;
import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.camera_width;

public class ViewPipeLine extends OpenCvPipeline {
    @Override
    public Mat processFrame(Mat input) {
        Bitmap map = Bitmap.createBitmap(camera_width, camera_height, Bitmap.Config.RGB_565);
        Utils.matToBitmap(input, map);

        FtcDashboard.getInstance().sendImage(map);

        return input;
    }
}
