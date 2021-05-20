package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.commandftc.opModes.CommandBasedTeleOp;
import org.firstinspires.ftc.teamcode.lib.kinematics.HolonomicOdometry;
import org.firstinspires.ftc.teamcode.lib.kinematics.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

@TeleOp(name="Odometry Test")
public class OdometryTest extends CommandBasedTeleOp {
    Odometry odometry;
    DriveTrainSubsystem driveTrain;

    @Override
    public void assign() {
        driveTrain = new DriveTrainSubsystem();

        odometry = new HolonomicOdometry(
                driveTrain::getLeftOdometryDistance,
                driveTrain::getRightOdometryDistance,
                driveTrain::getHorizontalOdometryDistance,
                Constants.DriveTrainConstants.kOdometryConstants.getVerticalWheelsDistance(),
                Constants.DriveTrainConstants.kOdometryConstants.getHorizontalWheelOffset()
        );


    }
}
