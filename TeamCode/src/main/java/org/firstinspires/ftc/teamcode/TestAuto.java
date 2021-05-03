package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.commandftc.RobotUniversal;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobellSubsystem;

import java.util.concurrent.BrokenBarrierException;

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

        vision.set_for_drive();

        waitForStart();

        while (vision.getError() != Constants.VisionConstants.camera_width/2.0 && opModeIsActive())
            ;

        align_robot_left(300);

//        while (opModeIsActive()) {
//            double error = -(-vision.getError());
//            telemetry.addData("error", error);
//            telemetry.update();
//        }
    }

    private void align_robot_left(double offset) {
        driveTrain.set_for_commands();
        while (opModeIsActive()) {
            double error = -(-vision.getError()+offset);
            driveTrain.driveLeft(-error/500);
            telemetry.addData("error", error);
            telemetry.update();
            if (Math.abs(error) <= 2) {
                driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                driveTrain.stop();
                break;
            }
        }
        driveTrain.set_for_autonomous();
    }
}
