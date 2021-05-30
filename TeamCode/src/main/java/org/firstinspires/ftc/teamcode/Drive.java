package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.commandftc.opModes.CommandBasedTeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.AlignRobotVisionCommand;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.DriveSideWaysCommand;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.DriveToDirectionCommand;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.TankDriveCommand;
import org.firstinspires.ftc.teamcode.commands.Intake.ManualIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.Shooter.SetShooterLiftCommand;
import org.firstinspires.ftc.teamcode.commands.Shooter.SetShooterSpeedCommand;
import org.firstinspires.ftc.teamcode.commands.Shooter.WaitForShooterCommand;
import org.firstinspires.ftc.teamcode.commands.Storage.AutomaticStorageCommand;
import org.firstinspires.ftc.teamcode.commands.Storage.ConstStorageCommand;
import org.firstinspires.ftc.teamcode.commands.Storage.ManualStorageCommand;
import org.firstinspires.ftc.teamcode.commands.Storage.SetStorageIndexCommand;
import org.firstinspires.ftc.teamcode.commands.Util.LoggerCommand;
import org.firstinspires.ftc.teamcode.commands.Wobell.KeepCurrentWobellPositionCommand;
import org.firstinspires.ftc.teamcode.commands.Wobell.OpenWobellCommand;
import org.firstinspires.ftc.teamcode.commands.Wobell.WobellLiftCommand;
import org.firstinspires.ftc.teamcode.commands.Wobell.WobellTargetPositionCommand;
import org.firstinspires.ftc.teamcode.lib.DashboardUtil;
import org.firstinspires.ftc.teamcode.lib.Util;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobellSubsystem;
import org.firstinspires.ftc.teamcode.vison.VisionTarget;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;

import static org.commandftc.RobotUniversal.opMode;

@TeleOp(name="Drive for Robotosh")
public class Drive extends CommandBasedTeleOp {
    protected DriveTrainSubsystem driveTrain;
    protected ShooterSubsystem shooter;
    protected IntakeSubsystem intake;
    protected StorageSubSystem storage;
    protected WobellSubsystem wobellSubsystem;
    protected VisionSubsystem vision;

    protected TankDriveCommand tankDriveCommand;
    protected DriveToDirectionCommand driveCommand;
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

    protected final VisionTarget visionTarget = VisionTarget.BlueTower;

    protected double drive_speed_modifier;

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

        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setLift(0.34);

        tankDriveCommand = new TankDriveCommand(driveTrain,
                () -> -gamepad1.left_stick_y * drive_speed_modifier, () -> -gamepad1.right_stick_y * drive_speed_modifier);
        driveCommand = new DriveToDirectionCommand(driveTrain,
                () -> -gamepad1.right_stick_y, () -> -gamepad1.right_stick_x, () -> -gamepad1.left_stick_x);
        driveSideWaysCommandCommand = new DriveSideWaysCommand(driveTrain,
                () -> Util.maxAbs(-gamepad1.left_stick_x * drive_speed_modifier, -gamepad1.right_stick_x * drive_speed_modifier));
        alignRobotCommand = new AlignRobotVisionCommand(driveTrain, vision);

        manualIntakeCommand = new ManualIntakeCommand(intake, () -> gamepad2.left_stick_y);

        automaticStorageCommand = new AutomaticStorageCommand(storage);
        manualStorageCommand = new ManualStorageCommand(storage, () -> gamepad2.right_stick_y);
        constStorageCommand = new ConstStorageCommand(storage);
        startStorageCommand = new SetStorageIndexCommand(storage, 0.7);

        startShooterCommand = new SetShooterSpeedCommand(shooter, 0.55);
        stopShooterCommand = new SetShooterSpeedCommand(shooter, 0);
        waitForShooterCommand = new WaitForShooterCommand(shooter, 2700);

        startShooterSequenceCommand = new SequentialCommandGroup(
                startShooterCommand,
                new InstantCommand(() -> manualIntakeCommand.setConstIntake(false)),
                waitForShooterCommand,
                new WaitCommand(0.4),
                new WaitCommand(0.7),
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
                new DriveForwardCommand(driveTrain, () -> -gamepad1.right_stick_y * drive_speed_modifier));
        gp1.right_stick_button().whenHeld(
                new DriveForwardCommand(driveTrain, () -> -gamepad1.left_stick_y * drive_speed_modifier));
        gp1.left_bumper().whileHeld(new DriveSideWaysCommand(driveTrain, () -> drive_speed_modifier));
        gp1.right_bumper().whileHeld(new DriveSideWaysCommand(driveTrain, () -> -drive_speed_modifier));
        gp1.x().whileHeld(alignRobotCommand);

        // ROBOT REVEAL CODE
//        gp1.a().whileHeld(new InstantCommand(() -> driveTrain.setPower(0.3), driveTrain)).whenReleased(new InstantCommand(()->driveTrain.setPower(0)));
        gp2.b().whileHeld(new InstantCommand(() -> shooter.setPower(0.55), shooter)).whenReleased(() -> shooter.setPower(0));

        new Button(() -> gamepad1.left_trigger > 0.1)
                .whenPressed(new InstantCommand(() -> drive_speed_modifier = 0.5))
                .whenReleased(new InstantCommand(() -> drive_speed_modifier = 0.8));

        new Button(() -> gamepad1.right_trigger > 0.1)
                .whenPressed(new InstantCommand(() -> drive_speed_modifier = 1))
                .whenReleased(new InstantCommand(() -> drive_speed_modifier = 0.8));
        drive_speed_modifier = 0.8;

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
        wobellSubsystem.setDefaultCommand(new WobellTargetPositionCommand(wobellSubsystem));

        new Button(() -> gamepad2.left_trigger > 0.1).whenHeld(new WobellLiftCommand(wobellSubsystem, () -> gamepad2.left_trigger)).whenReleased(new KeepCurrentWobellPositionCommand(wobellSubsystem));
        new Button(() -> gamepad2.right_trigger > 0.1).whenHeld(new WobellLiftCommand(wobellSubsystem, () -> -gamepad2.right_trigger)).whenReleased(new KeepCurrentWobellPositionCommand(wobellSubsystem));
        gp2.x().toggleWhenPressed(new OpenWobellCommand(wobellSubsystem));

        gp2.a().whenPressed(new InstantCommand(() -> shooter.setLift(Math.abs(shooter.getLift() - 0.35) < 0.001 ? 0.2 : 0.35), shooter));

        // Show time to make each loop
//        new LoopTimeCommand().schedule();
//        getLogCommand().schedule();

        new CommandBase() {
            private final Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
            @Override
            public void execute() {
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("left", shooter.getLeftVelocity());
                packet.put("right", shooter.getRightVelocity());
                packet.put("vx", Objects.requireNonNull(driveTrain.getPoseVelocity()).getX());
                DashboardUtil.drawRobot(packet.fieldOverlay(), driveTrain.getPoseEstimate());
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                telemetry.update();
            }
        }.schedule();
//        telemetry.addData("Runtime", this::getRuntime);
        telemetry.addData("Vision pipeline ms", vision.camera::getPipelineTimeMs);
//        telemetry.addData("Odometry", driveTrain::getPosition);
        telemetry.addData("Lift", shooter::getLift);
        telemetry.addData("pos", driveTrain::getPoseEstimate);
//        telemetry.addData("Wobell", wobellSubsystem::getCurrentPosition);
//        telemetry.addData("Wobell Lift", wobellSubsystem::getLift);
//        telemetry.addData("Shooter", shooter::getLeftVelocity);
//        telemetry.addData("gyro", driveTrain::getHeading);
        telemetry.addData("Vision error", vision::getError);
        telemetry.addData("ExternalHeading", driveTrain::getExternalHeading);
//        telemetry.addData("Vision target", vision::getTarget);
//        telemetry.addData("align active", alignRobotCommand::isScheduled);
//        telemetry.addData("color(red)", storage.getColorSensor()::red);
//        telemetry.addData("color(green)", storage.getColorSensor()::green);
//        telemetry.addData("color(blue)", storage.getColorSensor()::blue);

//        telemetry.addData("left(h)", () -> vision.align_pipeLine.left_rect == null ? 0 : vision.align_pipeLine.left_rect.y + vision.align_pipeLine.left_rect.height);
//        telemetry.addData("right(h)", () -> vision.align_pipeLine.right_rect == null ? 0 : vision.align_pipeLine.right_rect.y + vision.align_pipeLine.right_rect.height);

//        telemetry.addData("Storage", storage::getEncoder);
//        telemetry.addData("has ring", storage::seeing_ring);

//        telemetry.addData("RL", driveTrain::getRearLeftEncoder);
//        telemetry.addData("RR", driveTrain::getRearRightEncoder);
//        telemetry.addData("FL", driveTrain::getFrontLeftEncoder);
//        telemetry.addData("FR", driveTrain::getFrontRightEncoder);

//        telemetry.addData("Left", driveTrain::getLeftOdometryEncoder);
//        telemetry.addData("Right", driveTrain::getRightOdometryEncoder);
//        telemetry.addData("Horizontal", driveTrain::getHorizontalOdometryEncoder);

        telemetry.addData("velocity", () -> (driveTrain.getPoseVelocity() != null ? driveTrain.getPoseVelocity() : "null"));
//        telemetry.addData("loc", driveTrain.getLocalizer().getClass().getSimpleName());

        driveTrain.setPoseEstimate(new Pose2d(1.8288 , 1.8288));
    }

    @NotNull
    @Contract(" -> new")
    private Command getLogCommand() {
        Map<String, Supplier<Object>> map = new LinkedHashMap<>();

        map.put("time", this::getRuntime);
        map.put("shooter left", shooter::getLeftVelocity);
        map.put("shooter right", shooter::getRightVelocity);
        map.put("shooter lift", shooter::getLift);

        map.put("index", storage::getIndex);
        map.put("index(busy)", storage::isBusy);
        map.put("index(current)", storage::getTarget);

        map.put("wobell target", wobellSubsystem::getTargetPosition);
        map.put("wobell current", wobellSubsystem::getCurrentPosition);
        map.put("wobell power", wobellSubsystem::getPower);

        map.put("FL(current)", driveTrain::getFrontLeftEncoder);
        map.put("FR(current)", driveTrain::getFrontRightEncoder);
        map.put("RL(current)", driveTrain::getRearLeftEncoder);
        map.put("RR(current)", driveTrain::getRearRightEncoder);

        map.put("FL(target)", driveTrain::getFrontLeftTarget);
        map.put("FR(target)", driveTrain::getFrontRightTarget);
        map.put("RL(target)", driveTrain::getRearLeftTarget);
        map.put("RR(target)", driveTrain::getRearRightTarget);

        map.put("driveTrain(isBusy)", driveTrain::isBusy);

        map.put("Vision(error)", vision::getError);
        map.put("Vision(target)", vision::getTarget);

//        return map;
        List<String> entries_name = new ArrayList<>();
        List<Supplier<Object>> suppliers = new ArrayList<>();

        for (Map.Entry<String, Supplier<Object>> entry : map.entrySet()) {
            entries_name.add(entry.getKey());
            suppliers.add(entry.getValue());
        }

        return new LoggerCommand(entries_name, suppliers);
    }


}
