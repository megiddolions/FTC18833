package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.commandftc.opModes.CommandBasedTeleOp;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.TankDriveCommand;
import org.firstinspires.ftc.teamcode.lib.kinematics.RoadRunnerOdometry;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

@TeleOp(name="Odometry Test")
public class OdometryTest extends CommandBasedTeleOp {
    ThreeTrackingWheelLocalizer odometry;
    DriveTrainSubsystem driveTrain;

    @Override
    public void assign() {
        driveTrain = new DriveTrainSubsystem();

        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        odometry = new RoadRunnerOdometry(new DoubleSupplier[]{driveTrain::getLeftOdometryDistance, driveTrain::getRightOdometryDistance, driveTrain::getHorizontalOdometryDistance});

        new CommandBase() {
            @Override
            public void execute() {
                odometry.update();
            }
        }.schedule();

        driveTrain.setDefaultCommand(new TankDriveCommand(driveTrain, () -> -gamepad1.left_stick_y, () -> -gamepad1.right_stick_y));

        telemetry.addData("pos", odometry::getPoseEstimate);
//        telemetry.addData("pos3", driveTrain::getPosition);
        telemetry.addData("left", driveTrain::getLeftOdometryDistance);
        telemetry.addData("right", driveTrain::getRightOdometryDistance);
        telemetry.addData("center", driveTrain::getHorizontalOdometryEncoder);
    }
}
