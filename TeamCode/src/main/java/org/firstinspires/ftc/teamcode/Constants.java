package org.firstinspires.ftc.teamcode;

import java.util.function.DoubleFunction;

public final class Constants {

    public final static class MotorConstants {
        public final static class REV_HD_HEX {
            public final static double ticks_per_revolution = 28;
        }
    }

    public final static class ShooterConstants {
        public final static double kTicks_per_revolution = MotorConstants.REV_HD_HEX.ticks_per_revolution;

        public static double ticks_per_second_to_RPM(double ticks_per_second) {
            return ticks_per_second / kTicks_per_revolution * 60;
        }

        public static double RPM_to_ticks_per_second(double RPM) {
            return RPM / 60 * kTicks_per_revolution;
        }

        public static final String kLeftShooterName = "LeftShooter";
        public static final String kRightShooterName = "RightShooter";
        public static final String kIndexShooterName = "IndexMotor";
    }

    public final static class NetworkConstants {
        public final static String computer_ip = "192.168.49.72";
        public final static int server_port = 5038;
    }

    public final static class RuntimeConstants {
        final static boolean send_to_port = true;
    }

    public final static class VisionConstants {
        public final static String vuforiaLicenseKey = "AW1+wmn/////AAABmVmAOtGuak1MhR7DqLc8ekQmFHpf83mtlDBBKk3x5gDp5700GpPgEu9pdiHdtQiCYB5rXm+j9TmYHCdTQEqLBDg1lTruDLihYhJ3xDDS6OScGPpPWJGpPOWfW/zjylyhfd3+gxexOLoY0ZlTVe8m9e3pBj9LrSFHtNmissbwm1oN89E5sdW04hC/ccXdwvcz83TNd6d928ZeOH/yXXLB1BIVEF09F1N/fmH591eBYI2DAgtzkBrZW/6Xj+Up8yv6BJDtGzvepPJJzEJ39iwYVEVixG+AS8Q8dJbaqBIHZqf33s/g6z+eWGtlzC96T3BwOeevIeYpBtKCYYA+AoHoRpqWvLqpPqzzYOV3S7wSCs/b";

        public final static int camera_width = 1280;
        public final static int camera_height = 800;
    }
}
