package org.firstinspires.ftc.teamcode.commands.DriveTrain;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.commandftc.RobotUniversal;
import org.firstinspires.ftc.teamcode.lib.Util;
import org.firstinspires.ftc.teamcode.lib.kinematics.TankDrive;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.util.Iterator;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

@Config
public class AlignPowerShootsCommand extends CommandBase {
    private final TankDrive driveTrain;
    private final VisionSubsystem vision;
    private final Iterator<VoltageSensor> voltageSensor;
    private final PIDController pid = new PIDController(pidCoefficients.p, pidCoefficients.i, pidCoefficients.d);
    public static PIDFCoefficients pidCoefficients = new PIDFCoefficients(0.008, 0.001,0, 0);
    private final double offset;
    private double voltage = 14;

    public AlignPowerShootsCommand(TankDrive driveTrain, VisionSubsystem vision) {
        this(driveTrain, vision, 0);
    }

    public AlignPowerShootsCommand(TankDrive driveTrain, VisionSubsystem vision, double offset) {
        this.driveTrain = driveTrain;
        this.vision = vision;
        this.offset = offset;

        voltageSensor = RobotUniversal.hardwareMap.voltageSensor.iterator();

        addRequirements(driveTrain, vision);
    }

    @Override
    public void execute() {
        if (voltageSensor.hasNext())
            voltage = voltageSensor.next().getVoltage();
        double range = voltage < 12.5 ? 0.18 : 0.15;
        double out = Util.clamp(pid.calculate(vision.getError() + offset) ,  range, -range);
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
        return pid.atSetpoint() || Math.abs(vision.getError()) <= 25 && Math.abs(pid.getVelocityError()) <= 10;
    }
}
