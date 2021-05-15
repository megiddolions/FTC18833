package org.firstinspires.ftc.teamcode.commands.DriveTrain.autonomus;

import org.firstinspires.ftc.teamcode.lib.kinematics.TankDrive;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.vison.VisionTarget;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AlignPowerShootsCommand extends CommandBase {
    private final TankDrive driveTrain;
    private final VisionSubsystem vision;
    private final VisionTarget.PowerShoot target;


    public AlignPowerShootsCommand(TankDrive driveTrain, VisionSubsystem vision, VisionTarget.PowerShoot target) {
        this.driveTrain = driveTrain;
        this.vision = vision;
        this.target = target;

        addRequirements(driveTrain, vision);
    }

    @Override
    public void initialize() {
        vision.setPowerShootTarget(target);
    }



    @Override
    public void end(boolean interrupted) {
        driveTrain.stop();
    }
}
