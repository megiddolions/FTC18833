package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.commandftc.opModes.CommandBasedTeleOp;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.TankDriveCommand;
import org.firstinspires.ftc.teamcode.lib.kinematics.HolonomicOdometry;
import org.firstinspires.ftc.teamcode.lib.kinematics.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

@TeleOp(name="Odometry Test")
public class OdometryTest extends CommandBasedTeleOp {
    Odometry odometry;
    DriveTrainSubsystem driveTrain;

    @Override
    public void assign() {
        driveTrain = new DriveTrainSubsystem();

        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        odometry = new HolonomicOdometry(
                driveTrain::getLeftOdometryDistance,
                driveTrain::getRightOdometryDistance,
                driveTrain::getHorizontalOdometryDistance,
                Constants.DriveTrainConstants.kOdometryConstants.getVerticalWheelsDistance(),
                Constants.DriveTrainConstants.kOdometryConstants.getHorizontalWheelOffset()
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
