package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.commandftc.opModes.CommandBasedTeleOp;
import org.firstinspires.ftc.teamcode.commands.Util.LoopTimeCommand;
import org.firstinspires.ftc.teamcode.commands.Wobell.OpenWobellCommand;
import org.firstinspires.ftc.teamcode.commands.Wobell.WobellTargetPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobellSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;

@TeleOp(name="TestDrive")
public class TestDrive extends CommandBasedTeleOp {
    protected DriveTrainSubsystem driveTrain;
    protected ShooterSubsystem shooter;
    protected IntakeSubsystem intake;
    protected StorageSubSystem storage;
    protected WobellSubsystem wobellSubsystem;
    protected VisionSubsystem vision;

    @Override
    public void assign() {
        driveTrain = new DriveTrainSubsystem();
        shooter = new ShooterSubsystem();
        intake = new IntakeSubsystem();
        storage = new StorageSubSystem();
        wobellSubsystem = new WobellSubsystem();
//        vision = new VisionSubsystem();

//        vision.set_for_drive();

        wobellSubsystem.setDefaultCommand(new WobellTargetPositionCommand(wobellSubsystem));

        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setLift(0.375);

        new LoopTimeCommand().schedule();

        gp1.dpad_up().whenPressed(new InstantCommand(() -> wobellSubsystem.setTargetPosition(wobellSubsystem.getTargetPosition() - 500)));
        gp1.dpad_down().whenPressed(new InstantCommand(() -> wobellSubsystem.setTargetPosition(wobellSubsystem.getTargetPosition() + 500)));

        gp1.x().toggleWhenPressed(new OpenWobellCommand(wobellSubsystem));

        telemetry.addData("target", wobellSubsystem::getTargetPosition);
        telemetry.addData("current", wobellSubsystem::getCurrentPosition);
    }
}
