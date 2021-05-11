package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.commandftc.RobotUniversal;
import org.commandftc.opModes.LinearOpModeWithCommands;
import org.firstinspires.ftc.teamcode.commands.Wobell.WobellTargetPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobellSubsystem;
import org.jetbrains.annotations.NotNull;

@Deprecated
@Disabled
@Autonomous(name = "ÂùŤő")
public class Auto extends LinearOpModeWithCommands {
    protected DriveTrainSubsystem driveTrain;
    protected ShooterSubsystem shooter;
    protected IntakeSubsystem intake;
    protected StorageSubSystem storage;
    protected WobellSubsystem wobellSubsystem;
    protected VisionSubsystem vision;

    private long state;
    private int rings;

    @Override
    public void init_subsystems() {
        driveTrain = new DriveTrainSubsystem();
        shooter = new ShooterSubsystem();
        intake = new IntakeSubsystem();
        storage = new StorageSubSystem();
        wobellSubsystem = new WobellSubsystem();
        vision = new VisionSubsystem();

//        addSubsystems(driveTrain, shooter, intake, storage, wobellSubsystem, vision);
    }

    @Override
    public void runOpMode() {
        state = 0;
        RobotUniversal.telemetry = telemetry;
        RobotUniversal.opMode = this;
        RobotUniversal.hardwareMap = hardwareMap;

        wobellSubsystem.setDefaultCommand(new WobellTargetPositionCommand(wobellSubsystem));

        driveTrain.set_for_autonomous();
        storage.set_for_autonomous();
        vision.set_for_autonomous();

        wobellSubsystem.close();
        shooter.setLift(0.27);

//        while (opModeIsActive() && !driveTrain.isGyroCalibrated())
//            ;

        telemetry.addData("time", this::getRuntime);
        telemetry.addData("state", () -> state);
//        telemetry.addData("Target", vuforia::Visible_Target);
        telemetry.addData("lift", shooter::getLift);
        telemetry.addData("shooter(velocity)", shooter::getLeftVelocity);
//        telemetry.addData("Storage", storage::getEncoder);
//        telemetry.addData("RL", driveTrain::getRearLeftEncoder);
//        telemetry.addData("RR", driveTrain::getRearRightEncoder);
//        telemetry.addData("FL", driveTrain::getFrontLeftEncoder);
//        telemetry.addData("FR", driveTrain::getFrontRightEncoder);
//        telemetry.addData("wobell", wobellSubsystem::getCurrentPosition);
        telemetry.addData("gyro", () -> driveTrain.getHeading().getDegrees());
        telemetry.update();

        waitForStart();

        // Count rings
        rings = vision.count_rings();
        double pixels = vision.getOrangePixels();
        telemetry.addData("orange pixels", () -> pixels);
        telemetry.addData("rings", () -> rings);
        telemetry.update();

//        vision.set_for_drive();

        // Move to position to shoot power shoots
        shooter.setPower(0.475);
        driveTrain.setPower(0.7);
        driveForward(-2000);
        spinLeft(10);
        driveTrain.setPower(1);
        driveLeft(30);
        // Wait for shooter
        wait_for_shooter(2000);
        // Shoot all rings
        shoot_ring();
        driveLeft(-45);
        shoot_ring();
        driveLeft(-45);
        shoot_ring();
        shooter.setPower(0);
        wobellSubsystem.setTargetPosition(4100);

        // Act according to the amount of rings
        switch (rings) {
            case 0:
                wobell_A();
                break;
            case 1:
                wobell_B();
                break;
            case 4:
                wobell_C();
                break;
        }
    }

    private void wobell_A() {
        // Drive to A
        spinLeft(590);
        driveForward(-1400);
        // Unload wobell
        while (wobellSubsystem.isBusy() && opModeIsActive())
            ;
        wobellSubsystem.open();
        wobellSubsystem.setTargetPosition(3500);
        // drive backward
        driveForward(400);
        // Drive to pickup second
        wobellSubsystem.setTargetPosition(4500);
        driveTrain.setPower(0.7);
        spinLeft(690);
        driveForward(-1450);
        // Pickup second wobell
        wobellSubsystem.close();
        sleep(500);
        wobellSubsystem.setLift(0.15);
        // Drive to A again
        driveForward(1550);
        // Drop the second wobell
        wobellSubsystem.setLift(0);
        spinLeft(-680);
        wobellSubsystem.open();
        // Go back
        driveForward(300);
    }

    private void wobell_B() {
        // Go to square B
        spinLeft(300);
        driveForward(-1000);
        // Drop first wobell
        while (wobellSubsystem.isBusy() && opModeIsActive())
            ;
        wobellSubsystem.open();
        wobellSubsystem.setTargetPosition(3500);
        // Go to pick up the ring
        driveForward(200);
        spinLeft(-310);
        driveLeft(70);
        // Pickup ring
        intake.intake(1);
        storage.set_for_commands();
        storage.index(1);
        shooter.setLift(0.27);
        shooter.setPower(0.7);
        driveForward(1400);
        // shoot ring
        sleep(2000);
        shooter.setPower(0);
        storage.index(0);
        // Spin toward the second wobell
        wobellSubsystem.setTargetPosition(4500);
        spinLeft(1100);
        storage.set_for_autonomous();
        // Go to the second wobell
        driveForward(-400);
        wobellSubsystem.close();
        sleep(500);
//        spinLeft(310);
        driveForward(1550);
        spinLeft(-1400);
        driveForward(300);
        wobellSubsystem.open();
    }

    private void wobell_C() {
        state = 10;
        shooter.setLift(0.27);
        // Got to C
        spinLeft(320);
        driveForward(-2100);
        // Put first Wobell
        wobellSubsystem.open();
        wobellSubsystem.setTargetPosition(0);
        // Move to ring group
        driveTrain.setPower(1);
        spinLeft(-240);
        driveForward(1600);
        spinLeft(-100);
        shooter.setPower(0.5);
        driveForward(400);
        intake.intake(1);
        storage.set_for_commands();
        storage.index(1);
        driveTrain.setPower(0.25);
        driveForward(700);
        sleep(500);
        storage.index(0);
        driveTrain.setPower(1);
        shooter.setLift(0.35);
        driveForward(-1350);
        storage.index(1);
        sleep(1999);

//        intake.intake(1);
//        storage.set_for_commands();
//        driveForward(2400, new AutomaticStorageCommand(storage));
//
//        shooter.setPower(0.5);
//        // Collect rings
//        driveTrain.setPower(0.4);
//        intake.intake(1);
////        driveForward(1000, new AutomaticStorageCommand(storage));
//        storage.index(1);
//        driveForward(-1300);
//        // restore shooter

    }

    private void driveForward(double mm) {
        state++;
        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveTrain.driveForwardDistance(mm);
        while (driveTrain.isBusy() && opModeIsActive())
            ;
    }

    private void spinLeft(double spin) {
        state++;
        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        driveTrain.spinLeftDistance(spin);
        while (driveTrain.isBusy() && opModeIsActive())
            ;
    }

    private void driveLeft(double mm) {
        state++;
        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        driveTrain.driveLeftDistance(mm);
        while (driveTrain.isBusy() && opModeIsActive())
            ;
    }

    private void driveDiagonalLeft(double mm) {
        state++;
        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        driveTrain.driveDiagonalLeft(mm);
        while (driveTrain.isBusy() && opModeIsActive())
            ;
    }

    private void driveDiagonalRight(double mm) {
        state++;
        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        driveTrain.driveDiagonalRight(mm);
        while (driveTrain.isBusy() && opModeIsActive())
            ;
    }

    private void shoot_ring() {
        state++;
        storage.index_distance(105);
        while (storage.isBusy() && opModeIsActive())
            ;
    }

    private void wait_for_shooter(double velocity) {
        state++;
        while (shooter.getLeftVelocity() <= velocity && opModeIsActive())
            ;
//        sleep(1000);
    }

    private void align_robot_left(double offset) {
        driveTrain.set_for_commands();
        while (opModeIsActive()) {
            double error = -(-vision.getError()+offset);
            driveTrain.driveLeft(error/500);
            telemetry.addData("error", error);
            if (Math.abs(error) <= 2)
                break;
        }
        driveTrain.stop();
        driveTrain.set_for_autonomous();
    }

    private void align_gyro_to_zero(double offset) {
        driveTrain.set_for_commands();
        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        while (opModeIsActive()) {
            double error = -driveTrain.getHeading().getDegrees() + offset;
            driveTrain.setPower(-error / 35, error / 35);

            if (Math.abs(error) <= 1) {
                driveTrain.stop();
                break;
            }
            telemetry.update();
        }
        driveTrain.set_for_autonomous();
    }


}
