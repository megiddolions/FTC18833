package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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
import org.firstinspires.ftc.teamcode.vison.VisionTarget;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;

@TeleOp(name="Drive for Robotosh")
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

    protected SequentialCommandGroup startShooterSequenceCommand;
    protected Command stopShooterSequenceCommand;

    private final VisionTarget visionTarget = VisionTarget.BlueTower;

    @Override
    public void assign() {
        telemetry.addData("state", "DriveTrain");telemetry.update();
        driveTrain = new DriveTrainSubsystem();
        telemetry.addData("state", "shooter");telemetry.update();
        shooter = new ShooterSubsystem();
        telemetry.addData("state", "intake");telemetry.update();
        intake = new IntakeSubsystem();
        telemetry.addData("state", "storage");telemetry.update();
        storage = new StorageSubSystem();
        telemetry.addData("state", "wobellSubsystem");telemetry.update();
        wobellSubsystem = new WobellSubsystem();
        telemetry.addData("state", "vision");telemetry.update();
        vision = new VisionSubsystem();

        telemetry.addData("state", "vision for drive");telemetry.update();
        vision.setTarget(visionTarget);
        vision.update_align_pipeline();

        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setLift(0.375);

        tankDriveCommand = new TankDriveCommand(driveTrain,
                () -> -gamepad1.left_stick_y, () -> -gamepad1.right_stick_y);
        driveSideWaysCommandCommand = new DriveSideWaysCommand(driveTrain,
                () -> Util.maxAbs(-gamepad1.left_stick_x, -gamepad1.right_stick_x));
        alignRobotCommand = new AlignRobotVisionCommand(driveTrain, vision, visionTarget);

        manualIntakeCommand = new ManualIntakeCommand(intake, () -> gamepad2.left_stick_y);

        automaticStorageCommand = new AutomaticStorageCommand(storage);
        manualStorageCommand = new ManualStorageCommand(storage, () -> gamepad2.right_stick_y);
        constStorageCommand = new ConstStorageCommand(storage);
        startStorageCommand = new SetStorageIndexCommand(storage, 1);

        startShooterCommand = new SetShooterSpeedCommand(shooter, 0.55);
        stopShooterCommand = new SetShooterSpeedCommand(shooter, 0);
        waitForShooterCommand = new WaitForShooterCommand(shooter, 2500);

        startShooterSequenceCommand = new SequentialCommandGroup(
                startShooterCommand,
                waitForShooterCommand,
                startStorageCommand,
                new WaitCommand(5)
//                new InstantCommand(() -> shooter.setPower(0))
        );

        stopShooterSequenceCommand = stopShooterCommand;

        raiseShooterCommand = new SetShooterLiftCommand(shooter, 0.025);
        lowerShooterCommand = new SetShooterLiftCommand(shooter, -0.025);

        // DriveTrain
        driveTrain.setDefaultCommand(tankDriveCommand);
        gp1.left_stick_button().whenHeld(
                new DriveForwardCommand(driveTrain, () -> -gamepad1.right_stick_y));
        gp1.right_stick_button().whenHeld(
                new DriveForwardCommand(driveTrain, () -> -gamepad1.left_stick_y));
        gp1.left_bumper().whileHeld(new DriveSideWaysCommand(driveTrain, () -> 1));
        gp1.right_bumper().whileHeld(new DriveSideWaysCommand(driveTrain, () -> -1));
        gp1.y().whenPressed(new InstantCommand(() -> driveTrain.reset_encoders()));
        gp1.x().whenHeld(alignRobotCommand);

        new Button(() -> gamepad1.left_trigger > 0.1)
                .whenPressed(new InstantCommand(() -> driveTrain.drive_speed = 0.5))
                .whenReleased(new InstantCommand(() -> driveTrain.drive_speed = 0.8));

        new Button(() -> gamepad1.right_trigger > 0.1)
                .whenPressed(new InstantCommand(() -> driveTrain.drive_speed = 1))
                .whenReleased(new InstantCommand(() -> driveTrain.drive_speed = 0.8));

        // Intake
        intake.setDefaultCommand(manualIntakeCommand);
        new Button(() -> gamepad2.left_stick_button).whenPressed(
                new InstantCommand(() -> {
                    manualIntakeCommand.toggleConstIntake();
                    manualIntakeCommand.setConstIntakePower(gamepad2.left_stick_y);
                }));
        new Button(() -> gamepad2.left_stick_y < -0.5)
                .whenPressed(new InstantCommand(() -> manualIntakeCommand.setConstIntake(false), intake));

        // Storage
        storage.setDefaultCommand(manualStorageCommand);
        // Shooter
        gp2.left_bumper().whenPressed(startShooterSequenceCommand
        ).whenReleased(stopShooterSequenceCommand);
        
        // Shooter lift
        gp2.dpad_up().whenPressed(raiseShooterCommand);
        gp2.dpad_down().whenPressed(lowerShooterCommand);

        // wobell
        new Button(() -> gamepad2.left_trigger > 0.1).whenHeld(new WobellLiftCommand(wobellSubsystem, () -> gamepad2.left_trigger));
        new Button(() -> gamepad2.right_trigger > 0.1).whenHeld(new WobellLiftCommand(wobellSubsystem, () -> -gamepad2.right_trigger));
        gp2.x().toggleWhenPressed(new OpenWobellCommand(wobellSubsystem));

        gp2.a().whenPressed(new InstantCommand(() -> shooter.setLift(shooter.getLift() == 0.375 ? 0.2 : 0.375), shooter));

        // Show time to make each loop
        new LoopTimeCommand().schedule();

        telemetry.addData("Runtime", this::getRuntime);
        telemetry.addData("Odometry", driveTrain.odometry::getPosition);
        telemetry.addData("Lift", shooter::getLift);
//        telemetry.addData("Wobell", wobellSubsystem::getCurrentPosition);
//        telemetry.addData("Wobell Lift", wobellSubsystem::getLift);
        telemetry.addData("Shooter", shooter::getLeftVelocity);
//        telemetry.addData("gyro", driveTrain::getHeading);
        telemetry.addData("Vision error", vision::getError);
        telemetry.addData("Vision target", vision::getTarget);
        telemetry.addData("Vision pipeline ms", vision.camera::getPipelineTimeMs);

//        telemetry.addData("left(h)", () -> vision.align_pipeLine.left_rect == null ? 0 : vision.align_pipeLine.left_rect.y + vision.align_pipeLine.left_rect.height);
//        telemetry.addData("right(h)", () -> vision.align_pipeLine.right_rect == null ? 0 : vision.align_pipeLine.right_rect.y + vision.align_pipeLine.right_rect.height);

//        telemetry.addData("Storage", storage::getEncoder);
//        telemetry.addData("has ring", storage::seeing_ring);

        telemetry.addData("RL", driveTrain::getRearLeftEncoder);
        telemetry.addData("RR", driveTrain::getRearRightEncoder);
        telemetry.addData("FL", driveTrain::getFrontLeftEncoder);
        telemetry.addData("FR", driveTrain::getFrontRightEncoder);
    }
}
