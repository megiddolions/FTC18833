package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.commandftc.opModes.CommandBasedTeleOp;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.TankDriveCommand;
import org.firstinspires.ftc.teamcode.lib.kinematics.HolonomicOdometry;
import org.firstinspires.ftc.teamcode.lib.kinematics.Odometry;
import org.firstinspires.ftc.teamcode.lib.kinematics.RoadRunnerOdometry;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

@TeleOp(name="Odometry Test")
public class OdometryTest extends CommandBasedTeleOp {
    RoadRunnerOdometry odometry;
    DriveTrainSubsystem driveTrain;

    @Override
    public void assign() {
        driveTrain = new DriveTrainSubsystem();

        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        odometry = new RoadRunnerOdometry(
                new Pose2d[]{
                        new Pose2d(Constants.DriveTrainConstants.kOdometryConstants.leftWheel, Rotation2d.fromDegrees(0)),
                        new Pose2d(Constants.DriveTrainConstants.kOdometryConstants.rightWheel, Rotation2d.fromDegrees(0)),
                        new Pose2d(Constants.DriveTrainConstants.kOdometryConstants.horizontalWheel, Rotation2d.fromDegrees(90))
                },
                new DoubleSupplier[]{
                    driveTrain::getLeftOdometryDistance,
                    driveTrain::getRightOdometryDistance,
                    driveTrain::getHorizontalOdometryDistance
                }
        );

        new CommandBase() {
            @Override
            public void execute() {
                odometry.updatePose();
            }
        }.schedule();

        driveTrain.setDefaultCommand(new TankDriveCommand(driveTrain, () -> -gamepad1.left_stick_y, () -> -gamepad1.right_stick_y));

        telemetry.addData("pos", odometry::getPose);
        telemetry.addData("pos3", driveTrain::getPosition);
    }
}
