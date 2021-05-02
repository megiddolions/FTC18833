package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.commandftc.Command;
import org.commandftc.RobotUniversal;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Function;
import org.firstinspires.ftc.teamcode.commands.Storage.AutomaticStorageCommand;
import org.firstinspires.ftc.teamcode.commands.Storage.ManualStorageCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobellSubsystem;
import org.jetbrains.annotations.NotNull;

@Autonomous(name = "Auto")
public class Auto extends LinearOpMode {
    protected DriveTrainSubsystem driveTrain;
    protected ShooterSubsystem shooter;
    protected IntakeSubsystem intake;
    protected StorageSubSystem storage;
    protected WobellSubsystem wobellSubsystem;
    protected VisionSubsystem vision;

    private long state;
    private int rings;

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

        wobellSubsystem.close();
        shooter.setLift(0.20);

        driveTrain.set_for_autonomous();
        storage.set_for_autonomous();

        telemetry.addData("time", this::getRuntime);
        telemetry.addData("state", () -> state);
//        telemetry.addData("Target", vuforia::Visible_Target);
//        telemetry.addData("lift", shooter::getLift);
//        telemetry.addData("shooter(velocity)", shooter::getLeftVelocity);
        telemetry.addData("Storage", storage::getEncoder);
//        telemetry.addData("RL", driveTrain::getRearLeftEncoder);
//        telemetry.addData("RR", driveTrain::getRearRightEncoder);
//        telemetry.addData("FL", driveTrain::getFrontLeftEncoder);
//        telemetry.addData("FR", driveTrain::getFrontRightEncoder);
        telemetry.update();

        waitForStart();

        // Count rings
        rings = vision.count_rings();
        telemetry.addData("rings", () -> rings);
        telemetry.update();
        // Move to position to shoot power shoots
        shooter.setPower(0.55);
        driveTrain.setPower(0.5);
        driveForward(-2000);
        sleep(200);
        driveTrain.setPower(1);
        driveLeft(20);
        wait_for_shooter(2500);
        spinAbsoluteGyro(0);
        // Shoot all rings
        shoot_ring();
        driveLeft(-45);
        shoot_ring();
        driveLeft(-45);
        shoot_ring();
        shooter.setPower(0);
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
        spinLeft(610);
        driveForward(-1500);
        wobellSubsystem.setLift(-1);
        sleep(1300);
        wobellSubsystem.open();
        sleep(100);
        wobellSubsystem.setLift(1);
        sleep(350);
        wobellSubsystem.setLift(0);
        driveForward(400);
        driveTrain.setPower(0.7);
        spinLeft(710);
        driveForward(-1500);
        wobellSubsystem.close();
        sleep(500);
        wobellSubsystem.setLift(0.35);
        driveForward(1550);
        wobellSubsystem.setLift(0);
        spinLeft(-700);
        wobellSubsystem.setLift(-0.7);
        sleep(700);
        wobellSubsystem.open();
        wobellSubsystem.setLift(0);
        sleep(300);
        driveForward(300);
    }

    private void wobell_B() {
        spinLeft(300);
        driveForward(-1000);
        wobellSubsystem.setLift(-1);
        sleep(1300);
        wobellSubsystem.open();
        sleep(100);
        wobellSubsystem.setLift(1);
        sleep(1000);
        wobellSubsystem.setLift(0);
        driveForward(80);
    }

    private void wobell_C() {
        state = 10;
        // Got to C
        spinLeft(320);
        driveForward(-2100);
        // Put first Wobell
        wobellSubsystem.setLift(-1);
        sleep(1300);
        wobellSubsystem.open();
        sleep(100);
        wobellSubsystem.setLift(1);
        sleep(1000);
        wobellSubsystem.setLift(0);
        // Move to ring group
        driveTrain.setPower(1);
        spinLeft(-240);
        intake.intake(1);
        driveForward(1600);
        spinLeft(-100);
        wait_for_shooter(2500);
        // Start shooter wheels
        shooter.setPower(0.55);
        //

        // Collect rings
        driveTrain.setPower(0.4);
        intake.intake(1);
        storage.set_for_commands();
//        driveForward(1000, new AutomaticStorageCommand(storage));
        storage.index(1);
        driveForward(1300);
        // restore shooter
        while (opModeIsActive())
            ;
    }

    private void driveForward(double mm) {
        state++;
        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveTrain.driveForwardDistance(mm);
        while (driveTrain.isBusy() && opModeIsActive()) {
            telemetry.update();
        }
    }

    private void driveForward(double mm, @NotNull Command command) {
        state++;
        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveTrain.driveForwardDistance(mm);
        while (driveTrain.isBusy() && opModeIsActive()) {
            command.execute();
            telemetry.update();
        }
    }

    private void spinLeft(double spin) {
        state++;
        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        driveTrain.spinLeftDistance(spin);
        while (driveTrain.isBusy() && opModeIsActive())
            telemetry.update();
    }

    private void spinLeftGyro(double angle) {
        double target = driveTrain.getHeading().getDegrees() + angle;
        driveTrain.set_for_commands();
        double error = target - driveTrain.getHeading().getDegrees();
        while (Math.abs(error) <= 3) {
            driveTrain.setPower(error / 50, -error / 50);
            error = target - driveTrain.getHeading().getDegrees();
            telemetry.update();
        }
        driveTrain.set_for_autonomous();
    }
    private void spinAbsoluteGyro(double angle) {
        driveTrain.set_for_commands();
        double error = angle - driveTrain.getHeading().getDegrees();
        while (Math.abs(error) <= 3) {
            driveTrain.setPower(error / 20, -error / 20);
            error = angle - driveTrain.getHeading().getDegrees();
            telemetry.update();
        }
        driveTrain.setPower(0);
        driveTrain.set_for_autonomous();
    }

    private void driveLeft(double mm) {
        state++;
        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        driveTrain.driveLeftDistance(mm);
        while (driveTrain.isBusy() && opModeIsActive())
            telemetry.update();
    }

    private void driveDiagonalLeft(double mm) {
        state++;
        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        driveTrain.driveDiagonalLeft(mm);
        while (driveTrain.isBusy() && opModeIsActive())
            telemetry.update();
    }

    private void driveDiagonalRight(double mm) {
        state++;
        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        driveTrain.driveDiagonalRight(mm);
        while (driveTrain.isBusy() && opModeIsActive())
            telemetry.update();
    }

    private void shoot_ring() {
        state++;
        storage.index_distance(105);
        while (storage.isBusy() && opModeIsActive())
            telemetry.update();
    }

    private void wait_for_shooter(double velocity) {
        state++;
        while (shooter.getLeftVelocity() <= velocity && opModeIsActive())
            telemetry.update();
//        sleep(1000);
    }
}
