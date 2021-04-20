package org.firstinspires.ftc.teamcode.lib;

import android.annotation.SuppressLint;

import java.text.SimpleDateFormat;
import java.util.Date;

public final class Util {
    @SuppressLint("SimpleDateFormat")
    public String getTime() {
        return new SimpleDateFormat("yyyy-MM-dd HH:mm:ss").format(new Date());
    }

    public static double clamp(double v, double max, double min) {
        return Math.max(min, Math.min(max, v));
    }
}
