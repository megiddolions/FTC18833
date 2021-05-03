package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.commandftc.Command;
import org.commandftc.RobotUniversal;
import org.commandftc.Subsystem;
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

import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;

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

    private final Set<Subsystem> subsystems = new HashSet<>();

    private void addSubsystems(Subsystem ... subsystems) {
        Collections.addAll(this.subsystems, subsystems);
    }

    private void update() {
        for (Subsystem subsystem : subsystems) {
            subsystem.periodic();
        }
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        subsystems.clear();
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

        addSubsystems(driveTrain, shooter, intake, storage, wobellSubsystem, vision);

        driveTrain.set_for_autonomous();
        storage.set_for_autonomous();
        vision.set_for_autonomous();

        wobellSubsystem.close();
        shooter.setLift(0.27);

        telemetry.addData("time", this::getRuntime);
        telemetry.addData("state", () -> state);
//        telemetry.addData("Target", vuforia::Visible_Target);
//        telemetry.addData("lift", shooter::getLift);
        telemetry.addData("shooter(velocity)", shooter::getLeftVelocity);
//        telemetry.addData("Storage", storage::getEncoder);
//        telemetry.addData("RL", driveTrain::getRearLeftEncoder);
//        telemetry.addData("RR", driveTrain::getRearRightEncoder);
//        telemetry.addData("FL", driveTrain::getFrontLeftEncoder);
//        telemetry.addData("FR", driveTrain::getFrontRightEncoder);
        telemetry.addData("orange pixels", vision::getOrangePixels);
        telemetry.addData("wobell", wobellSubsystem::getCurrentPosition);
        telemetry.update();

        waitForStart();

        // Count rings
        rings = vision.count_rings();
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
        wait_for_shooter(2000);
        // Shoot all rings
        shoot_ring();
        driveLeft(-45);
        shoot_ring();
        driveLeft(-45);
        shoot_ring();
        shooter.setPower(0);
        wobellSubsystem.setTargetPosition(4500);
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
            update();
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
        spinLeft(300);
        driveForward(-1000);
        while (wobellSubsystem.isBusy() && opModeIsActive())
            update();
        wobellSubsystem.open();
        wobellSubsystem.setTargetPosition(3500);
        driveForward(200);
        //ido's part
        spinLeft(-310);
        driveLeft(70);
        intake.intake(1);
        storage.set_for_commands();
        storage.index(1);
        shooter.setLift(0.27);
        shooter.setPower(0.7);
        driveForward(1400);
        sleep(2000);
        shooter.setPower(0);
        storage.index(0);
        spinLeft(1200);
        storage.set_for_autonomous();
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
            update();
        }
    }

    private void driveForward(double mm, @NotNull Command command) {
        state++;
        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveTrain.driveForwardDistance(mm);
        while (driveTrain.isBusy() && opModeIsActive()) {
            command.execute();
            update();
        }
    }

    private void spinLeft(double spin) {
        state++;
        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        driveTrain.spinLeftDistance(spin);
        while (driveTrain.isBusy() && opModeIsActive())
            update();
    }

    private void driveLeft(double mm) {
        state++;
        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        driveTrain.driveLeftDistance(mm);
        while (driveTrain.isBusy() && opModeIsActive())
            update();
    }

    private void driveDiagonalLeft(double mm) {
        state++;
        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        driveTrain.driveDiagonalLeft(mm);
        while (driveTrain.isBusy() && opModeIsActive())
            update();
    }

    private void driveDiagonalRight(double mm) {
        state++;
        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        driveTrain.driveDiagonalRight(mm);
        while (driveTrain.isBusy() && opModeIsActive())
            update();
    }

    private void shoot_ring() {
        state++;
        storage.index_distance(105);
        while (storage.isBusy() && opModeIsActive())
            update();
    }

    private void wait_for_shooter(double velocity) {
        state++;
        while (shooter.getLeftVelocity() <= velocity && opModeIsActive())
            update();
//        sleep(1000);
    }

    private void align_robot_left(double offset) {
        driveTrain.set_for_commands();
        while (opModeIsActive()) {
            double error = -(-vision.getError()+offset);
            driveTrain.driveLeft(error/500);
            telemetry.addData("error", error);
            update();
            if (Math.abs(error) <= 2)
                break;
        }
        driveTrain.stop();
        driveTrain.set_for_autonomous();
    }
}
