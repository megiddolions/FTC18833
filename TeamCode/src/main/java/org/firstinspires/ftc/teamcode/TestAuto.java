package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.commandftc.RobotUniversal;
import org.commandftc.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobellSubsystem;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

//@Disabled
@Autonomous(name="Test Auto")
public class TestAuto extends LinearOpMode {
    protected DriveTrainSubsystem driveTrain;
    protected ShooterSubsystem shooter;
    protected IntakeSubsystem intake;
    protected StorageSubSystem storage;
    protected WobellSubsystem wobellSubsystem;
    protected VisionSubsystem vision;

    private Set<Subsystem> subsystems = new HashSet<>();

    private void addSubsystems(Subsystem... subsystems) {
        Collections.addAll(this.subsystems, subsystems);
    }

    private void update() {
        for (Subsystem subsystem : subsystems) {
            subsystem.periodic();
        }
        telemetry.update();
    }
    @Override
    public void runOpMode() {
        subsystems.clear();
        RobotUniversal.telemetry = telemetry;
        RobotUniversal.opMode = this;
        RobotUniversal.hardwareMap = hardwareMap;

        driveTrain = new DriveTrainSubsystem();
        shooter = new ShooterSubsystem();
        intake = new IntakeSubsystem();
        storage = new StorageSubSystem();
        wobellSubsystem = new WobellSubsystem();
        vision = new VisionSubsystem();

        addSubsystems(driveTrain, shooter, intake, storage, wobellSubsystem, vision);
        wobellSubsystem.close();

        telemetry.addData("wobell", wobellSubsystem::getCurrentPosition);

        waitForStart();

        wobellSubsystem.setTargetPosition(3000);

        while (opModeIsActive()) {
            update();
        }

    }
}
