package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.commandftc.opModes.CommandBasedTeleOp;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.AlignPowerShootsCommand;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.AlignWobbleVisionCommand;
import org.firstinspires.ftc.teamcode.commands.DriveTrain.TankDriveCommand;
import org.firstinspires.ftc.teamcode.commands.Wobell.KeepCurrentWobellPositionCommand;
import org.firstinspires.ftc.teamcode.commands.Wobell.OpenWobellCommand;
import org.firstinspires.ftc.teamcode.commands.Wobell.WobellLiftCommand;
import org.firstinspires.ftc.teamcode.commands.Wobell.WobellTargetPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;
import org.firstinspires.ftc.teamcode.vison.pipelines.align.BluePowerShootsAlignPipeLine;
import org.firstinspires.ftc.teamcode.vison.pipelines.align.VisionTarget;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;

@TeleOp(name="Vision Test")
public class VisionTest extends CommandBasedTeleOp {
    private DriveTrainSubsystem driveTrain;
    private VisionSubsystem vision;
    private WobbleSubsystem wobbleSubsystem;

    private BluePowerShootsAlignPipeLine powerShootsAlignPipeLine;

    private TankDriveCommand tankDriveCommand;
    private AlignPowerShootsCommand alignRobotCommand;

    @Override
    public void assign() {
        driveTrain = new DriveTrainSubsystem();
        vision = new VisionSubsystem();
        wobbleSubsystem = new WobbleSubsystem();

        powerShootsAlignPipeLine = new BluePowerShootsAlignPipeLine();

        tankDriveCommand = new TankDriveCommand(driveTrain, () -> -gamepad1.left_stick_y * (gamepad1.left_trigger > 0.5 ? 1 : 0.5), () -> -gamepad1.right_stick_y * (gamepad1.left_trigger > 0.5 ? 1 : 0.5));
        alignRobotCommand = new AlignPowerShootsCommand(driveTrain, vision);

//        vision.setAlignPipeLine(powerShootsAlignPipeLine);
        vision.setTarget(VisionTarget.BlueWobble);

        driveTrain.setDefaultCommand(tankDriveCommand);

        assign_buttons();
    }

    public void assign_buttons() {
        gp1.dpad_left().whenPressed(new InstantCommand(() -> powerShootsAlignPipeLine.target = powerShootsAlignPipeLine.target.left()));
        gp1.dpad_right().whenPressed(new InstantCommand(() -> powerShootsAlignPipeLine.target = powerShootsAlignPipeLine.target.right()));

        // wobble
        wobbleSubsystem.setDefaultCommand(new WobellTargetPositionCommand(wobbleSubsystem));

        new Button(() -> gamepad2.left_trigger > 0.1).whenHeld(new WobellLiftCommand(wobbleSubsystem, () -> gamepad2.left_trigger)).whenReleased(new KeepCurrentWobellPositionCommand(wobbleSubsystem));
        new Button(() -> gamepad2.right_trigger > 0.1).whenHeld(new WobellLiftCommand(wobbleSubsystem, () -> -gamepad2.right_trigger)).whenReleased(new KeepCurrentWobellPositionCommand(wobbleSubsystem));
        gp2.x().toggleWhenPressed(new OpenWobellCommand(wobbleSubsystem));

//        gp1.y().whenHeld(alignRobotCommand);
        gp1.y().whenHeld(new AlignWobbleVisionCommand(driveTrain, vision));
    }
}
