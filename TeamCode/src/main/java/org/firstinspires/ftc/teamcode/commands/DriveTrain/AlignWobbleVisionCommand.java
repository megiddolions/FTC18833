package org.firstinspires.ftc.teamcode.commands.DriveTrain;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.lib.kinematics.TankDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

@Config
public class AlignWobbleVisionCommand extends CommandBase {
    private final TankDrive driveTrain;
    private final VisionSubsystem vision;
    private PIDController pid;
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0.001, 0, 0);

    public AlignWobbleVisionCommand(TankDrive driveTrain, VisionSubsystem vision) {
        this.driveTrain = driveTrain;
        this.vision = vision;

        addRequirements(driveTrain, vision);
    }

    @Override
    public void initialize() {
        pid = new PIDController(pidCoefficients);
    }

    @Override
    public void execute() {
        double out = pid.calculate(vision.getError());
        driveTrain.tankDrive(out, -out);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stop();
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint() || Math.abs(pid.getPositionError()) < 120 && Math.abs(pid.getVelocityError()) <= 10;
    }
}
