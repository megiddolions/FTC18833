package org.firstinspires.ftc.teamcode.commands.DriveTrain;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.lib.Util;
import org.firstinspires.ftc.teamcode.lib.kinematics.TankDrive;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

@Config
public class AlignPowerShootsCommand extends CommandBase {
    private final TankDrive driveTrain;
    private final VisionSubsystem vision;
    private final PIDController pid = new PIDController(pidCoefficients.p, pidCoefficients.i, pidCoefficients.d);
    public static PIDFCoefficients pidCoefficients = new PIDFCoefficients(0.008, 0.001,0, 0);
    private final double offset;

    public AlignPowerShootsCommand(TankDrive driveTrain, VisionSubsystem vision) {
        this(driveTrain, vision, 0);
    }

    public AlignPowerShootsCommand(TankDrive driveTrain, VisionSubsystem vision, double offset) {
        this.driveTrain = driveTrain;
        this.vision = vision;
        this.offset = offset;

        addRequirements(driveTrain, vision);
    }

    @Override
    public void execute() {
        double out = Util.clamp(pid.calculate(vision.getError() + offset) , 0.15, -0.15);
        driveTrain.tankDrive(out, -out);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stop();
        pid.reset();
    }

    @Override
    public boolean isFinished() {
//        return pid.atSetpoint();
        return pid.atSetpoint() || Math.abs(vision.getError()) <= 25;
    }
}
