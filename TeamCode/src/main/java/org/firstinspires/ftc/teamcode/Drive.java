package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.commandftc.Command;
import org.commandftc.Trigger;
import org.commandftc.opModes.CommandBasedTeleOp;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.AlignRobotVuforiaCommand;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.DriveSideWaysCommand;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.TankDriveCommand;
import org.firstinspires.ftc.teamcode.commands.Intake.StartIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.Shooter.SetShooterLiftCommand;
import org.firstinspires.ftc.teamcode.commands.Shooter.SetShooterSpeedCommand;
import org.firstinspires.ftc.teamcode.commands.Storage.AutomaticStorageCommand;
import org.firstinspires.ftc.teamcode.commands.Storage.ManualStorageCommand;
import org.firstinspires.ftc.teamcode.lib.Util;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VuforiaSubsystem;

@TeleOp(name="drive")
public class Drive extends CommandBasedTeleOp {
    protected DriveTrainSubsystem driveTrain;
    protected ShooterSubsystem shooter;
    protected IntakeSubsystem intake;
    protected StorageSubSystem storage;
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
        vision = new VisionSubsystem();
        vuforia = new VuforiaSubsystem();

        addSubsystems(driveTrain, shooter, intake, storage, vision, vuforia);

        tankDriveCommand = new TankDriveCommand(driveTrain,
                () -> -gamepad1.left_stick_y, () -> -gamepad1.right_stick_y);
        driveSideWaysCommandCommand = new DriveSideWaysCommand(driveTrain,
                () -> Util.maxAbs(-gamepad1.left_stick_x, -gamepad1.right_stick_x));
        alignRobotCommand = new AlignRobotVuforiaCommand(driveTrain, vuforia);

        startIntakeCommand = new StartIntakeCommand(intake, () -> gamepad2.left_stick_y);

        automaticStorageCommand = new AutomaticStorageCommand(storage);
        manualStorageCommand = new ManualStorageCommand(storage, () -> -gamepad2.right_stick_y);

        startShooterCommand = new SetShooterSpeedCommand(shooter, storage, 0.5);
        raiseShooterCommand = new SetShooterLiftCommand(shooter, 0.05);
        lowerShooterCommand = new SetShooterLiftCommand(shooter, -0.05);

        // DriveTrain
        driveTrain.setDefaultCommand(tankDriveCommand);
        new Trigger(() -> Math.abs(Util.maxAbs(gamepad1.left_stick_x, gamepad1.right_stick_x)) >
                Math.abs(Util.maxAbs(gamepad1.left_stick_y, gamepad1.right_stick_y)))
                .whileHeld(driveSideWaysCommandCommand);
        new Trigger(() -> gamepad1.x).whileHeld(alignRobotCommand);

        // Intake
        intake.setDefaultCommand(startIntakeCommand);

        // Storage
        storage.setDefaultCommand(automaticStorageCommand);
        new Trigger(() -> Math.abs(gamepad2.right_stick_y) > 0.2).whileHeld(manualStorageCommand);

        // Shooter
        new Trigger(() -> gamepad2.x).whenHeld(startShooterCommand);
        // Shooter lift
        new Trigger(() -> gamepad2.dpad_up).whenPressed(raiseShooterCommand);
        new Trigger(() -> gamepad2.dpad_down).whenPressed(lowerShooterCommand);

        telemetry.addData("Distance", vuforia::distance);
        telemetry.addData("Lift", shooter::getLift);
        telemetry.addData("Shooter", shooter::getLeftVelocity);
        telemetry.addData("Runtime", this::getRuntime);
    }
}
