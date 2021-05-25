package org.firstinspires.ftc.teamcode.commands.DriveTrain;

import org.firstinspires.ftc.teamcode.lib.kinematics.TankDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.vison.VisionTarget;
import org.firstinspires.ftc.teamcode.vison.pipelines.align.AlignPipeLine;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AlignRobotVisionCommand extends CommandBase {
    private final TankDrive driveTrain;
    private final VisionSubsystem vision;
//    private final PIDController pid = new PIDController(0.001,0,0.00001);
    private final PIDController pid = new PIDController(0.001, 0.000001,0);
    private final double offset;

    public AlignRobotVisionCommand(TankDrive driveTrain, VisionSubsystem vision) {
        this(driveTrain, vision, 0);
    }

    public AlignRobotVisionCommand(TankDrive driveTrain, VisionSubsystem vision, double offset) {
        this.driveTrain = driveTrain;
        this.vision = vision;
        this.offset = offset;

        addRequirements(driveTrain, vision);
    }

    @Override
    public void execute() {
        double out = pid.calculate(vision.getError()+offset);
        driveTrain.tankDrive(out, -out);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stop();
        pid.reset();
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }
}
