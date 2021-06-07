package org.firstinspires.ftc.teamcode.commands.DriveTrain;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.lib.Util;
import org.firstinspires.ftc.teamcode.lib.kinematics.TankDrive;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

@Config
public class AlignTowerCommand extends CommandBase {
    private final TankDrive driveTrain;
    private final VisionSubsystem vision;
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0.02, 0, 0);
    public PIDController pidController;

    public AlignTowerCommand(TankDrive driveTrain, VisionSubsystem vision) {
        this.driveTrain = driveTrain;
        this.vision = vision;

        addRequirements(driveTrain, vision);
    }

    @Override
    public void initialize() {
        pidController = new PIDController(pidCoefficients);
    }

    @Override
    public void execute() {
        double out = Util.clamp(pidController.calculate(vision.getError()), 0.2, -0.2);
        driveTrain.tankDrive(out, -out);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stop();
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint() || Math.abs(pidController.getPositionError()) <= 70 && Math.abs(pidController.getVelocityError()) <= 10;
    }
}
