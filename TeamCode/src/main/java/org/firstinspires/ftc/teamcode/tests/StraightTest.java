package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.commandftc.RobotUniversal;
import org.commandftc.opModes.LinearOpModeWithCommands;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.TankDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "tests")
public class StraightTest extends LinearOpModeWithCommands {
    public static double DISTANCE = 1; // meters
    private DriveTrainSubsystem driveTrain;


    @Override
    public void init_subsystems() {
        driveTrain = new DriveTrainSubsystem();
    }

    @Override
    public void runOpMode() {
        RobotUniversal.hardwareMap = hardwareMap;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

//        new TankDriveCommand(driveTrain, () -> gamepad1.left_stick_y, () -> gamepad1.right_stick_y).schedule();

        Trajectory trajectory = driveTrain.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        driveTrain.followTrajectoryAsync(trajectory);

        while (driveTrain.isBusy() && opModeIsActive())
            ;

        Pose2d poseEstimate = driveTrain.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}