package org.firstinspires.ftc.teamcode.commands.Wobell;

import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;
import org.firstinspires.ftc.teamcode.Constants.WobellConstants;

import edu.wpi.first.wpilibj2.command.PIDCommand;

public class WobellTargetPositionCommand extends PIDCommand {

    public WobellTargetPositionCommand(WobbleSubsystem wobbleSubsystem) {
        super(WobellConstants.kPid,
                wobbleSubsystem::getTargetPosition,
                wobbleSubsystem::getCurrentPosition,
                wobbleSubsystem::setLift,
                wobbleSubsystem);
    }
}
