package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.commandftc.opModes.CommandBasedTeleOp;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

@Disabled
@TeleOp(name="Odometry Test")
public class OdometryTest extends CommandBasedTeleOp {
    DriveTrainSubsystem driveTrain;

    @Override
    public void assign() {
        driveTrain = new DriveTrainSubsystem();

        telemetry.addData("position", driveTrain::getPosition);
        telemetry.addData("left", driveTrain::getLeftOdometryEncoder);
        telemetry.addData("right", driveTrain::getRightOdometryEncoder);
    }
}
