package org.firstinspires.ftc.teamcode.subsystems;

import org.commandftc.Subsystem;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import static org.commandftc.RobotUniversal.hardwareMap;
import static org.commandftc.RobotUniversal.telemetry;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.mmPerInch;

public class VuforiaSubsystem extends Subsystem {
    private final List<VuforiaTrackable> allTrackballs = new ArrayList<>();
    private Orientation rotation;
    private VectorF translation;

    public VuforiaSubsystem() {
        WebcamName cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int camera_id = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(camera_id);

        parameters.vuforiaLicenseKey = Constants.VisionConstants.vuforiaLicenseKey;

        /*
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = cameraName;

        // Make sure extended tracking is disabled for this example.
        parameters.useExtendedTracking = false;

        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackables targetsUltimateGoal = vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");


        allTrackballs.addAll(targetsUltimateGoal);

        //Set the position of the perimeter targets with relation to origin (center of field)
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -(float)(72 * mmPerInch), (float)(6 * mmPerInch))
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, (float)(72 * mmPerInch), (float)(6 * mmPerInch))
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-(float)(72 * mmPerInch), 0, (float)(6 * mmPerInch))
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation((float)(72 * mmPerInch), (float)(36 * mmPerInch), (float)(6 * mmPerInch))
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation((float)(72 * mmPerInch), -(float)(36 * mmPerInch), (float)(6 * mmPerInch))
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(0, 0, 0)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        for (VuforiaTrackable trackable : allTrackballs) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(cameraName, cameraLocationOnRobot);
        }

        targetsUltimateGoal.activate();
    }

    @Override
    public void periodic() {
        AtomicBoolean targetVisible = new AtomicBoolean(false);
        OpenGLMatrix lastLocation = null;
        for (VuforiaTrackable trackable : allTrackballs) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                visible_Target = trackable.getName();
                targetVisible.set(true);

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible.get() && lastLocation != null) {
            translation = lastLocation.getTranslation();
//            telemetry.addData("Pos (mm)", "{X, Y, Z} = %.1f, %.1f, %.1f",
//                    translation.get(0), translation.get(1), translation.get(2));

            // express the rotation of the robot in degrees.
            rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
//            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        }
        else {
            visible_Target = "none";
        }
    }
    private String visible_Target = "none";
    public String Visible_Target() {
        return visible_Target;
    }

    public boolean has_target() {
        return !allTrackballs.isEmpty() && rotation != null && translation != null;
    }

    public double heading() {
        if (translation != null)
            return rotation.thirdAngle;
        else
            return Double.POSITIVE_INFINITY;
    }

    public double distance() {
        if (translation != null)
        return translation.get(0);
        else
            return -1;
    }
}
