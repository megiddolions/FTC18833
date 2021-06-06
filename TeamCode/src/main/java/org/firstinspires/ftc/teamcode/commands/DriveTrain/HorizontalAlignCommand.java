package org.firstinspires.ftc.teamcode.commands.DriveTrain;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

@Config
public class HorizontalAlignCommand extends CommandBase {
    private final DriveTrainSubsystem driveTrain;
    private final VisionSubsystem vision;
    private PIDController pid;
    public static PIDCoefficients pidCoefficientsA = new PIDCoefficients(0.0015, 0,0);
    public static PIDCoefficients pidCoefficientsB = new PIDCoefficients(0.002, 0,0);

    public HorizontalAlignCommand(DriveTrainSubsystem driveTrain, VisionSubsystem vision) {
        this.driveTrain = driveTrain;
        this.vision = vision;

        addRequirements(driveTrain, vision);
    }

    @Override
    public void initialize() {
        if (Math.abs(vision.getFrontError()) >= 180) {
            pid = new PIDController(pidCoefficientsA);
        } else {
            pid = new PIDController(pidCoefficientsB);
        }
    }

    @Override
    public void execute() {
        double out = pid.calculate(vision.getFrontError());
        driveTrain.driveLeft(-out);
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint() || Math.abs(pid.getPositionError()) <= 40;
    }
}
