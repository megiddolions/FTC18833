package org.firstinspires.ftc.teamcode.opmods.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class BlueAuto extends DefaultAuto {
    @Override
    public void init_robot() {

    }

    @Override
    public Command getAutonomousCommand() {
        int rings = vision.count_rings();
        telemetry.addLine("rings: " + rings);
        FtcDashboard.getInstance().getTelemetry().addLine("rings: " + rings);
        FtcDashboard.getInstance().getTelemetry().addLine("pixels: " + vision.getOrangePixels());
        FtcDashboard.getInstance().getTelemetry().update();

        return new SequentialCommandGroup(
                getRingCommand(rings, new Pose2d())
        );
    }

    private Command getRingCommand(int rings, Pose2d end) {
        switch (rings) {
            case 0:
            case 1:
            case 4:
            default:
                return new InstantCommand();
        }
    }
}
