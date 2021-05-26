package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.lib.kinematics.OdometryConstants;

import java.util.function.DoubleFunction;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;

public final class Constants {

    public final static class MotorConstants {
        public final static class REV_HD_HEX {
            public final static double ticks_per_revolution = 28;
        }

        public final static class REV_CORE_HEX {
            public final static double ticks_per_revolution = 288;
        }

        public final static class REV_THROUGH_BORE_ENCODER {
            public final static double ticks_per_revolution = 8192;
        }
    }

    @Config
    public final static class  DriveTrainConstants {
        public final static double ticks_per_revolution =
                15 * MotorConstants.REV_HD_HEX.ticks_per_revolution;

        public final static MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
                new Translation2d(0.28, 0.34),
                new Translation2d(0.28, -0.34),
                new Translation2d(-0.28, 0.34),
                new Translation2d(-0.28, -0.34)
        );

        public final static double WheelRadios = 50;

        public final static DoubleFunction<Double> mm_to_ticks = (double mm) -> mm / WheelRadios / 2 / Math.PI * ticks_per_revolution;

        public static final OdometryConstants kOdometryConstants = new OdometryConstants(
//            new Translation2d(-0.0965, -0.11),
//            new Translation2d(0.0965, -0.11),
//            new Translation2d(0, 0.075),
            new Translation2d(0.095, -0.122),
            new Translation2d(-0.095, -0.122),
            new Translation2d(0, 0.005),
                0.06,
                MotorConstants.REV_THROUGH_BORE_ENCODER.ticks_per_revolution);

        public final static double kV = 0.7849;
        public final static double kStatic = 0.13083;
        public final static double kA = 0.19;
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

    public final static class IndexConstants {
        public final static double kTicks_per_revolution = MotorConstants.REV_HD_HEX.ticks_per_revolution;
        public final static double gear_ratio = 45 / 1.5;

        public final static double kWheelRadios = 15;

        public final static DoubleFunction<Double> mm_to_ticks = (double mm) ->
                mm / kWheelRadios / 2 / Math.PI * kTicks_per_revolution * gear_ratio;
    }

    public final static class WobellConstants {
        public static final PIDController kPid = new PIDController(0.001, 0, 0.00001);
    }

    public final static class NetworkConstants {
        public final static String computer_ip = "192.168.43.72";
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
    
//    public final static class OdometryConstants {
//        public final static double left_wheel_x = 0;
//        public final static double right_wheel_x = 0;
//        public final static double vertical_wheel_y = 0;
//        public final static double horizontal_wheel_y = 0;
//
//        public final static double vertical_wheels_difference = Math.abs(left_wheel_x - right_wheel_x);
//    }
}
