package org.firstinspires.ftc.teamcode.lib.kinematics;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;

import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;

public class RoadRunnerOdometry extends ThreeTrackingWheelLocalizer {
    private final DoubleSupplier[] encoders;
    public RoadRunnerOdometry(DoubleSupplier[] encoders) {
        super(Arrays.asList(
                new Pose2d(-0.122, 0.095, 0.0), // left parallel
                new Pose2d(-0.122, -0.095, 0.0), // right parallel
                new Pose2d(0.005, 0, Math.toRadians(-90)) // perpendicular
        ));
        this.encoders = encoders;
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(encoders[0].getAsDouble(), encoders[1].getAsDouble(), encoders[2].getAsDouble());
    }
}
