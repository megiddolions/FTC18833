package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.commandftc.Command;
import org.commandftc.InstantCommand;
import org.commandftc.Trigger;
import org.commandftc.opModes.CommandBasedTeleOp;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.AlignRobotVuforiaCommand;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.DriveSideWaysCommand;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.TankDriveCommand;
import org.firstinspires.ftc.teamcode.commands.Intake.StartIntakeCommand;
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
import org.firstinspires.ftc.teamcode.subsystems.VuforiaSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobellSubsystem;

@TeleOp(name="drive")
public class Drive extends CommandBasedTeleOp {
    protected DriveTrainSubsystem driveTrain;
    protected ShooterSubsystem shooter;
    protected IntakeSubsystem intake;
    protected StorageSubSystem storage;
    protected WobellSubsystem wobellSubsystem;
    protected VisionSubsystem vision;
    protected VuforiaSubsystem vuforia;

    protected TankDriveCommand tankDriveCommand;
    protected DriveSideWaysCommand driveSideWaysCommandCommand;
    protected Command alignRobotCommand; // method of aligning could changed

    protected StartIntakeCommand startIntakeCommand;

    protected AutomaticStorageCommand automaticStorageCommand;
    protected ManualStorageCommand manualStorageCommand;

    protected SetShooterSpeedCommand startShooterCommand;
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
        vuforia = new VuforiaSubsystem();

//        addSubsystems(driveTrain, shooter, intake, storage, wobellSubsystem, vision);
        addSubsystems(driveTrain, shooter, intake, storage, wobellSubsystem, vision, vuforia);

        tankDriveCommand = new TankDriveCommand(driveTrain,
                () -> -gamepad1.left_stick_y, () -> -gamepad1.right_stick_y);
        driveSideWaysCommandCommand = new DriveSideWaysCommand(driveTrain,
                () -> Util.maxAbs(-gamepad1.left_stick_x, -gamepad1.right_stick_x));
        alignRobotCommand = new AlignRobotVuforiaCommand(driveTrain, vuforia);

        startIntakeCommand = new StartIntakeCommand(intake, () -> gamepad2.left_stick_y);

        automaticStorageCommand = new AutomaticStorageCommand(storage);
        manualStorageCommand = new ManualStorageCommand(storage, () -> gamepad2.right_stick_y);

        startShooterCommand = new SetShooterSpeedCommand(shooter, storage, 0.65);
        raiseShooterCommand = new SetShooterLiftCommand(shooter, 0.05);
        lowerShooterCommand = new SetShooterLiftCommand(shooter, -0.05);

        // DriveTrain
        driveTrain.setDefaultCommand(tankDriveCommand);
//        new Trigger(() -> Math.abs(Util.maxAbs(gamepad1.left_stick_x, gamepad1.right_stick_x)) >
//                Math.abs(Util.maxAbs(gamepad1.left_stick_y, gamepad1.right_stick_y)))
//                .whileHeld(driveSideWaysCommandCommand);
        gp1.left_stick_button().whenHeld(
                new DriveForwardCommand(driveTrain, () -> -gamepad1.right_stick_y));
        gp1.right_stick_button().whenHeld(
                new DriveForwardCommand(driveTrain, () -> -gamepad1.left_stick_y));
        gp1.x().whileHeld(alignRobotCommand);
        gp1.left_bumper().whileHeld(new DriveSideWaysCommand(driveTrain, () -> 1));
        gp1.right_bumper().whileHeld(new DriveSideWaysCommand(driveTrain, () -> -1));
        gp1.y().whenPressed(new InstantCommand(() -> driveTrain.reset_encoders()));

        new Trigger(() -> gamepad1.left_trigger > 0.1)
                .whenPressed(new InstantCommand(() -> driveTrain.drive_speed = 0.5, driveTrain))
                .whenReleased(new InstantCommand(() -> driveTrain.drive_speed = 0.8, driveTrain));

        new Trigger(() -> gamepad1.right_trigger > 0.1)
                .whenPressed(new InstantCommand(() -> driveTrain.drive_speed = 1, driveTrain))
                .whenReleased(new InstantCommand(() -> driveTrain.drive_speed = 0.8, driveTrain));

        // Intake
        intake.setDefaultCommand(startIntakeCommand);

        // Storage
//        storage.setDefaultCommand(automaticStorageCommand);
        new Trigger(() -> Math.abs(gamepad2.right_stick_y) > 0.1).whileHeld(manualStorageCommand);

        // Shooter
//        gp2.x().whenHeld(startShooterCommand);
        gp2.left_bumper().whenPressed(new InstantCommand(() -> shooter.setPower(0.5), shooter))
                        .whenReleased(new InstantCommand(() -> shooter.setPower(0)));
        // Shooter lift
        gp2.dpad_up().whenPressed(raiseShooterCommand);
        gp2.dpad_down().whenPressed(lowerShooterCommand);

        // wobell
        new Trigger(() -> gamepad2.left_trigger > 0.1).whenHeld(new WobellLiftCommand(wobellSubsystem, () -> gamepad2.left_trigger));
        new Trigger(() -> gamepad2.right_trigger > 0.1).whenHeld(new WobellLiftCommand(wobellSubsystem, () -> -gamepad2.right_trigger));
        gp2.x().toggleWhenPressed(new OpenWobellCommand(wobellSubsystem));

        telemetry.addData("Runtime", this::getRuntime);
        telemetry.addData("Odometry", driveTrain.odometry::getPoseMeters);
//        telemetry.addData("Distance", vuforia::distance);
        telemetry.addData("Lift", shooter::getLift);
//        telemetry.addData("Wobell Lift", wobellSubsystem::getLift);
//        telemetry.addData("Shooter", shooter::getLeftVelocity);
//        telemetry.addData("Visible Target", vuforia::Visible_Target);
//        telemetry.addData("Angle", vuforia::heading);
//        telemetry.addData("Storage", storage::getEncoder);
//        telemetry.addData("has ring", storage::seeing_ring);

        telemetry.addData("RL", driveTrain::getRearLeftEncoder);
        telemetry.addData("RR", driveTrain::getRearRightEncoder);
        telemetry.addData("FL", driveTrain::getFrontLeftEncoder);
        telemetry.addData("FR", driveTrain::getFrontRightEncoder);


        /*
        constIntake = (0.9 < gamepad2.right_stick_y && gamepad2.right_stick_button) || constIntake;
        //Disable constant intake if right_stick_y is under -0.5
        constIntake = (!(gamepad2.right_stick_y < -0.5)) && constIntake;
        //Sets motor based on constant intake or right_stick_y
        intakeMotor.setPower((constIntake) ? 1 : gamepad2.right_stick_y);
         */
    }
}
