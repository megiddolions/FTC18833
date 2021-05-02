package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.commandftc.RobotUniversal;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobellSubsystem;

@Autonomous(name="Test Auto™")
public class TestAuto extends LinearOpMode {
    protected DriveTrainSubsystem driveTrain;
    protected ShooterSubsystem shooter;
    protected IntakeSubsystem intake;
    protected StorageSubSystem storage;
    protected WobellSubsystem wobellSubsystem;
    protected VisionSubsystem vision;

    private long state;
    boolean b = false;
    @Override
    public void runOpMode() {
        state = 0;
        RobotUniversal.telemetry = telemetry;
        RobotUniversal.opMode = this;
        RobotUniversal.hardwareMap = hardwareMap;

        driveTrain = new DriveTrainSubsystem();
        shooter = new ShooterSubsystem();
        intake = new IntakeSubsystem();
        storage = new StorageSubSystem();
        wobellSubsystem = new WobellSubsystem();
        vision = new VisionSubsystem();

        telemetry.addData("heading", driveTrain.getHeading()::getDegrees);
        waitForStart();
        spinLeftGyro(90);
        while (opModeIsActive()) {
            if (gamepad1.a) {
            }
            telemetry.update();
        }
    }

    private void spinLeftGyro(double angle) {
        double target = driveTrain.getHeading().getDegrees() + angle;
        driveTrain.set_for_commands();
        double error = target - driveTrain.getHeading().getDegrees();
        while (Math.abs(error) > 3 && opModeIsActive()) {
            driveTrain.setPower(-error / 50, error / 50);
            error = target - driveTrain.getHeading().getDegrees();
            telemetry.addData("error", error);
            telemetry.update();
        }
        driveTrain.set_for_autonomous();
    }
}
