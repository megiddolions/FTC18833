package org.firstinspires.ftc.teamcode.lib.kinematics;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class Odometry {
    private final OdometryConstants constants;
    private final DoubleSupplier leftEncoder;
    private final DoubleSupplier rightEncoder;
    private final DoubleSupplier horizontalEncoder;
    private double lastLeftPosition;
    private double lastRightPosition;
    private double lastVerticalPosition;

    private Pose2d position;

    public Odometry(OdometryConstants constants, DoubleSupplier leftEncoder, DoubleSupplier rightEncoder, DoubleSupplier horizontalEncoder) {
        this.constants = constants;
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.horizontalEncoder = horizontalEncoder;

        this.position = new Pose2d();

        updateLastPositions();
    }

    public Odometry(OdometryConstants constants, DoubleSupplier leftEncoder, DoubleSupplier rightEncoder, DoubleSupplier horizontalEncoder, Pose2d position) {
        this.constants = constants;
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.horizontalEncoder = horizontalEncoder;

        this.position = position;

        updateLastPositions();
    }

    void updateLastPositions() {
        lastLeftPosition = leftEncoder.getAsDouble();
        lastRightPosition = rightEncoder.getAsDouble();
        lastVerticalPosition = horizontalEncoder.getAsDouble();
    }

    public Pose2d getPosition() {
        return position;
    }

    public void update() {
        double deltaLeft = leftEncoder.getAsDouble() - lastLeftPosition;
        double deltaRight = rightEncoder.getAsDouble() - lastRightPosition;
        double deltaHorizontal = horizontalEncoder.getAsDouble() - lastVerticalPosition;

        double deltaAngle = (deltaLeft - deltaRight) / constants.getVerticalWheelsDistance();

        double totalLeft = lastLeftPosition + deltaLeft;
        double totalRight = lastRightPosition + deltaRight;

        double lastAngle = (lastLeftPosition - lastRightPosition) / constants.getVerticalWheelsDistance();

        double currentAngle = lastAngle + deltaAngle;

        Translation2d delta_position = new Translation2d(
                (deltaRight + deltaLeft) / 2.0,
                deltaHorizontal - deltaAngle);

        position = position.plus(new Transform2d(delta_position.rotateBy(new Rotation2d(currentAngle)), new Rotation2d(deltaAngle)));

        updateLastPositions();
    }
}
