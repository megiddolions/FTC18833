package org.firstinspires.ftc.teamcode.lib.kinematics;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class RoadRunnerOdometry {
    private Pose2d pos;
    private final DecompositionSolver forwardSolver;
    private final DoubleSupplier[] encoders;

    public RoadRunnerOdometry(Pose2d[] wheels, DoubleSupplier[] encoders) {
        this.encoders = encoders;
        List<Pose2d> wheels1 = Arrays.asList(wheels.clone());

        Array2DRowRealMatrix inverseMatrix = new Array2DRowRealMatrix(3, 3);
        for (int i = 0; i < 3; i++) {
            Translation2d orientationVector = new Translation2d(
                    wheels[i].getRotation().getSin(), wheels[i].getRotation().getCos());
            Translation2d positionVector = wheels[i].getTranslation();
            inverseMatrix.setEntry(i, 0, orientationVector.getX());
            inverseMatrix.setEntry(i, 1, orientationVector.getY());
            inverseMatrix.setEntry(
                    i,
                    2,
                    positionVector.getX() * orientationVector.getY() - positionVector.getX() * orientationVector.getY()
            );
        }

        forwardSolver = new LUDecomposition(inverseMatrix).getSolver();
    }

    @NotNull
    private Transform2d calculatePoseDelta(double[] wheels_delta) {
         RealMatrix rawPoseDelta = forwardSolver.solve(
                MatrixUtils.createRealMatrix(
                        new double[][]{wheels_delta}
                )
        );
        return new Transform2d(
                new Translation2d(
                        rawPoseDelta.getEntry(0, 0),
                        rawPoseDelta.getEntry(1, 0)
                ),
                new Rotation2d(rawPoseDelta.getEntry(2, 0))
        );
    }

    public void updatePose(Pose2d newPose) {
        pos = newPose;
    }

    public void updatePose() {
        Transform2d dp = calculatePoseDelta(new double[]{encoders[0].getAsDouble(), encoders[1].getAsDouble(), encoders[2].getAsDouble()});
        pos = pos.plus(dp);
    }

    public Pose2d getPose() {
        return pos;
    }
}
