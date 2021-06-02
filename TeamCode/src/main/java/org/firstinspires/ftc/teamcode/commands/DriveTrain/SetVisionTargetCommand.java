package org.firstinspires.ftc.teamcode.commands.DriveTrain;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.vison.pipelines.align.VisionTarget;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetVisionTargetCommand extends CommandBase {
    private final VisionSubsystem vision;
    private final VisionTarget target;

    public SetVisionTargetCommand(VisionSubsystem vision, VisionTarget target) {
        this.vision = vision;
        this.target = target;

        addRequirements(vision);
    }

    @Override
    public void initialize() {
        vision.setTarget(target);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
