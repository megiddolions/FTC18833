package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.commandftc.Command;
import org.commandftc.CommandScheduler;
import org.commandftc.InstantCommand;
import org.commandftc.ParallelCommand;
import org.commandftc.SequentialCommand;
import org.commandftc.Trigger;
import org.commandftc.opModes.CommandBasedTeleOp;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.AlignRobotVisionCommand;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.DriveSideWaysCommand;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.TankDriveCommand;
import org.firstinspires.ftc.teamcode.commands.Intake.ManualIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.Shooter.WaitForShooterCommand;
import org.firstinspires.ftc.teamcode.commands.Storage.ConstStorageCommand;
import org.firstinspires.ftc.teamcode.commands.Storage.SetStorageIndexCommand;
import org.firstinspires.ftc.teamcode.commands.Util.LoopTimeCommand;
import org.firstinspires.ftc.teamcode.commands.Util.WaitCommand;
import org.firstinspires.ftc.teamcode.commands.Wobell.OpenWobellCommand;
import org.firstinspires.ftc.teamcode.commands.Shooter.SetShooterLiftCommand;
import org.firstinspires.ftc.teamcode.commands.Shooter.SetShooterSpeedCommand;
import org.firstinspires.ftc.teamcode.commands.Storage.AutomaticStorageCommand;
import org.firstinspires.ftc.teamcode.commands.Storage.ManualStorageCommand;
import org.firstinspires.ftc.teamcode.commands.Wobell.WobellLiftCommand;
import org.firstinspires.ftc.teamcode.lib.Util;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobellSubsystem;

@TeleOp(name="drive")
public class Drive extends CommandBasedTeleOp {
    protected DriveTrainSubsystem driveTrain;
    protected ShooterSubsystem shooter;
    protected IntakeSubsystem intake;
    protected StorageSubSystem storage;
    protected WobellSubsystem wobellSubsystem;
    protected VisionSubsystem vision;

    protected TankDriveCommand tankDriveCommand;
    protected DriveSideWaysCommand driveSideWaysCommandCommand;
    protected AlignRobotVisionCommand alignRobotCommand;

    protected ManualIntakeCommand manualIntakeCommand;

    protected AutomaticStorageCommand automaticStorageCommand;
    protected ManualStorageCommand manualStorageCommand;
    protected ConstStorageCommand constStorageCommand;
    protected SetStorageIndexCommand startStorageCommand;

    protected SetShooterSpeedCommand startShooterCommand;
    protected SetShooterSpeedCommand stopShooterCommand;
    protected WaitForShooterCommand waitForShooterCommand;
    protected SetShooterLiftCommand raiseShooterCommand;
    protected SetShooterLiftCommand lowerShooterCommand;

    @Override
    public void assign() {

        driveTrain = new DriveTrainSubsystem();
        shooter = new ShooterSubsystem();
        intake = new IntakeSubsystem();
        storage = new StorageSubSystem();
        wobellSubsystem = new WobellSubsystem();
        vision = new VisionSubsystem();

        vision.set_for_drive();

        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setLift(0.375);

//        addSubsystems(driveTrain, shooter, intake, storage, wobellSubsystem);
        addSubsystems(driveTrain, shooter, intake, storage, wobellSubsystem, vision);

        tankDriveCommand = new TankDriveCommand(driveTrain,
                () -> -gamepad1.left_stick_y, () -> -gamepad1.right_stick_y);
        driveSideWaysCommandCommand = new DriveSideWaysCommand(driveTrain,
                () -> Util.maxAbs(-gamepad1.left_stick_x, -gamepad1.right_stick_x));
        alignRobotCommand = new AlignRobotVisionCommand(driveTrain, vision);

        manualIntakeCommand = new ManualIntakeCommand(intake, () -> gamepad2.left_stick_y);

        automaticStorageCommand = new AutomaticStorageCommand(storage);
        manualStorageCommand = new ManualStorageCommand(storage, () -> gamepad2.right_stick_y);
        constStorageCommand = new ConstStorageCommand(storage);
        startStorageCommand = new SetStorageIndexCommand(storage, 1);

        startShooterCommand = new SetShooterSpeedCommand(shooter, 0.70);
        stopShooterCommand = new SetShooterSpeedCommand(shooter, 0);

        waitForShooterCommand = new WaitForShooterCommand(shooter, 2500);

        raiseShooterCommand = new SetShooterLiftCommand(shooter, 0.025);
        lowerShooterCommand = new SetShooterLiftCommand(shooter, -0.025);

        // DriveTrain
        driveTrain.setDefaultCommand(tankDriveCommand);
//        new Trigger(() -> Math.abs(Util.maxAbs(gamepad1.left_stick_x, gamepad1.right_stick_x)) >
//                Math.abs(Util.maxAbs(gamepad1.left_stick_y, gamepad1.right_stick_y)))
//                .whileHeld(driveSideWaysCommandCommand);
        gp1.left_stick_button().whenHeld(
                new DriveForwardCommand(driveTrain, () -> -gamepad1.right_stick_y));
        gp1.right_stick_button().whenHeld(
                new DriveForwardCommand(driveTrain, () -> -gamepad1.left_stick_y));
        gp1.left_bumper().whileHeld(new DriveSideWaysCommand(driveTrain, () -> 1));
        gp1.right_bumper().whileHeld(new DriveSideWaysCommand(driveTrain, () -> -1));
        gp1.y().whenPressed(new InstantCommand(() -> driveTrain.reset_encoders()));
//        gp1.x().whenHeld(alignRobotCommand);
//        gp1.dpad_left().whenHeld(new DriveLeftDistanceCommand(driveTrain, 45));
//        gp1.dpad_right().whenHeld(new DriveLeftDistanceCommand(driveTrain,-45));

        new Trigger(() -> gamepad1.left_trigger > 0.1)
                .whenPressed(new InstantCommand(() -> driveTrain.drive_speed = 0.5))
                .whenReleased(new InstantCommand(() -> driveTrain.drive_speed = 0.8));

        new Trigger(() -> gamepad1.right_trigger > 0.1)
                .whenPressed(new InstantCommand(() -> driveTrain.drive_speed = 1))
                .whenReleased(new InstantCommand(() -> driveTrain.drive_speed = 0.8));

        // Intake
        intake.setDefaultCommand(manualIntakeCommand);
        new Trigger(() -> gamepad2.left_stick_button).whenPressed(
                new InstantCommand(() -> {
                    manualIntakeCommand.toggleConstIntake();
                    manualIntakeCommand.setConstIntakePower(gamepad2.left_stick_y);
                }));
        new Trigger(() -> gamepad2.left_stick_y < -0.5)
                .whenPressed(new InstantCommand(() -> manualIntakeCommand.setConstIntake(false), intake));

        // Storage
//        storage.setDefaultCommand(automaticStorageCommand);
        storage.setDefaultCommand(manualStorageCommand);
//        new Trigger(() -> -gamepad1.right_stick_y < -0.2 && gamepad1.right_stick_button).whenPressed(
//                new InstantCommand(() -> constStorageCommand.power = --gamepad1.right_stick_y, storage)
//                        .andThen(constStorageCommand));
//        new Trigger(() -> -gamepad2.right_stick_y > 0.5).whenPressed(new InstantCommand(() -> constStorageCommand.power = 0, storage));

        // Shooter
//        gp2.left_bumper().whenPressed(new InstantCommand(() -> {
//                manualIntakeCommand.setConstIntake(false);
//                shooter.setPower(0.55);
//        }, shooter))
//                        .whenReleased(new InstantCommand(() -> shooter.setPower(0)));
        gp2.left_bumper().whenPressed(new SequentialCommand(
                new InstantCommand(() -> shooter.setPower(0.55), shooter)
//                waitForShooterCommand,
//                new InstantCommand(() -> manualIntakeCommand.setConstIntake(false), intake),
//                startStorageCommand,
//                new WaitCommand(4000)
        )).whenReleased(stopShooterCommand);
        
        // Shooter lift
        gp2.dpad_up().whenPressed(raiseShooterCommand);
        gp2.dpad_down().whenPressed(lowerShooterCommand);

        // wobell
        new Trigger(() -> gamepad2.left_trigger > 0.1).whenHeld(new WobellLiftCommand(wobellSubsystem, () -> gamepad2.left_trigger));
        new Trigger(() -> gamepad2.right_trigger > 0.1).whenHeld(new WobellLiftCommand(wobellSubsystem, () -> -gamepad2.right_trigger));
        gp2.x().toggleWhenPressed(new OpenWobellCommand(wobellSubsystem));

        gp2.a().whenPressed(new InstantCommand(() -> shooter.setLift(shooter.getLift() == 0.375 ? 0.2 : 0.375), shooter));

        // Show time to make each loop
        CommandScheduler.scheduleCommand(new LoopTimeCommand());

        telemetry.addData("Runtime", this::getRuntime);
//        telemetry.addData("Odometry", driveTrain.odometry::getPoseMeters);
//        telemetry.addData("Distance", vuforia::distance);
        telemetry.addData("Lift", shooter::getLift);
//        telemetry.addData("Wobell", wobellSubsystem::getCurrentPosition);
//        telemetry.addData("Wobell Lift", wobellSubsystem::getLift);
//        telemetry.addData("Shooter", shooter::getLeftVelocity);
//        telemetry.addData("gyro", driveTrain::getHeading);

//        telemetry.addData("left(h)", () -> vision.align_pipeLine.left_rect == null ? 0 : vision.align_pipeLine.left_rect.y + vision.align_pipeLine.left_rect.height);
//        telemetry.addData("right(h)", () -> vision.align_pipeLine.right_rect == null ? 0 : vision.align_pipeLine.right_rect.y + vision.align_pipeLine.right_rect.height);

//        telemetry.addData("Storage", storage::getEncoder);
//        telemetry.addData("has ring", storage::seeing_ring);

//        telemetry.addData("RL", driveTrain::getRearLeftEncoder);
//        telemetry.addData("RR", driveTrain::getRearRightEncoder);
//        telemetry.addData("FL", driveTrain::getFrontLeftEncoder);
//        telemetry.addData("FR", driveTrain::getFrontRightEncoder);

        /*
        constIntake = (0.9 < gamepad2.right_stick_y && gamepad2.right_stick_button) || constIntake;
        //Disable constant intake if right_stick_y is under -0.5
        constIntake = (!(gamepad2.right_stick_y < -0.5)) && constIntake;
        //Sets motor based on constant intake or right_stick_y
        intakeMotor.setPower((constIntake) ? 1 : gamepad2.right_stick_y);
         */
    }
}
