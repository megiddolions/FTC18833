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
}
