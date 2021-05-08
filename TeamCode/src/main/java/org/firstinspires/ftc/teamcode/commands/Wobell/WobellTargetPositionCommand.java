package org.firstinspires.ftc.teamcode.commands.Wobell;

import org.firstinspires.ftc.teamcode.subsystems.WobellSubsystem;
import org.firstinspires.ftc.teamcode.Constants.WobellConstants;

import edu.wpi.first.wpilibj2.command.PIDCommand;

public class WobellTargetPositionCommand extends PIDCommand {

    public WobellTargetPositionCommand(WobellSubsystem wobellSubsystem) {
        super(WobellConstants.kPid,
                wobellSubsystem::getTargetPosition,
                wobellSubsystem::getCurrentPosition,
                wobellSubsystem::setLift,
                wobellSubsystem);
    }
}
