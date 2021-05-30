package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class DashboardUtil {
    private static final double DEFAULT_RESOLUTION = 2.0; // distance units; presumed inches
    private static final double ROBOT_RADIUS = 9; // in


    public static void drawPoseHistory(Canvas canvas, List<Pose2d> poseHistory) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];
        for (int i = 0; i < poseHistory.size(); i++) {
            Pose2d pose = poseHistory.get(i);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawSampledPath(Canvas canvas, Path path, double resolution) {
        int samples = (int) Math.ceil(path.length() / resolution);
        double[] xPoints = new double[samples];
        double[] yPoints = new double[samples];
        double dx = path.length() / (samples - 1);
        for (int i = 0; i < samples; i++) {
            double displacement = i * dx;
            Pose2d pose = path.get(displacement);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawSampledPath(Canvas canvas, Path path) {
        drawSampledPath(canvas, path, DEFAULT_RESOLUTION);
    }

    private static Translation2d[] getRobotCorners(edu.wpi.first.wpilibj.geometry.Pose2d pose) {
        Translation2d[] corners = new Translation2d[4];

        corners[0] = pose.plus(
                new Transform2d(
                        new Translation2d(.2125, .205),
                        new Rotation2d())).getTranslation();

        corners[1] = pose.plus(
                new Transform2d(
                        new Translation2d(-.2125, .205),
                        new Rotation2d())).getTranslation();

        corners[2] = pose.plus(
                new Transform2d(
                        new Translation2d(-.2125, -.205),
                        new Rotation2d())).getTranslation();

        corners[3] = pose.plus(
                new Transform2d(
                        new Translation2d(.2125, -.205),
                        new Rotation2d())).getTranslation();

        return corners;
    }

    public static Translation2d[] getIntakeCorners(edu.wpi.first.wpilibj.geometry.Pose2d pose) {
        Translation2d[] corners = new Translation2d[4];

        corners[0] = pose.plus(
                new Transform2d(
                        new Translation2d(.26, .1835),
                        new Rotation2d())).getTranslation();

        corners[1] = pose.plus(
                new Transform2d(
                        new Translation2d(.31, .1835),
                        new Rotation2d())).getTranslation();

        corners[2] = pose.plus(
                new Transform2d(
                        new Translation2d(.31, -.1835),
                        new Rotation2d())).getTranslation();

        corners[3] = pose.plus(
                new Transform2d(
                        new Translation2d(.26, -.1835),
                        new Rotation2d())).getTranslation();

        return corners;
    }

    private static void draw_robot_base(Canvas canvas, edu.wpi.first.wpilibj.geometry.Pose2d pose) {
        Translation2d[] corners = getRobotCorners(pose);

        double[] raw_x = new double[]{corners[0].getX(), corners[1].getX(), corners[2].getX(), corners[3].getX()};
        double[] raw_y = new double[]{corners[0].getY(), corners[1].getY(), corners[2].getY(), corners[3].getY()};

        for (int i = 0; i < 4; i++) {
            raw_x[i] *= 39.3700787;
            raw_y[i] *= 39.3700787;

            raw_x[i] -= 6 * 12;
            raw_y[i] -= 6 * 12;
        }


        canvas.strokePolygon(raw_x, raw_y);

        canvas.fillCircle((pose.getX()  - 1.8288) * 3.6576 * 12,
                (pose.getY() - 1.8288) * 3.6576 * 12,1);
    }

    private static void draw_robot_intake(Canvas canvas, edu.wpi.first.wpilibj.geometry.Pose2d pose) {
        Translation2d[] corners = getIntakeCorners(pose);

        double[] raw_x = new double[]{corners[0].getX(), corners[1].getX(), corners[2].getX(), corners[3].getX()};
        double[] raw_y = new double[]{corners[0].getY(), corners[1].getY(), corners[2].getY(), corners[3].getY()};

        for (int i = 0; i < 4; i++) {
            raw_x[i] *= 39.3700787;
            raw_y[i] *= 39.3700787;

            raw_x[i] -= 6 * 12;
            raw_y[i] -= 6 * 12;
        }


        canvas.strokePolygon(raw_x, raw_y);
    }

    public static void drawRobot(Canvas canvas, Pose2d pose) {
        draw_robot_base(canvas, new edu.wpi.first.wpilibj.geometry.Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(pose.getHeading())));
        draw_robot_intake(canvas, new edu.wpi.first.wpilibj.geometry.Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(pose.getHeading())));
    }
}