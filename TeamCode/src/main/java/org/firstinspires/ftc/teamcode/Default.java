package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.lib.CommandBaseOpMode;
import org.firstinspires.ftc.teamcode.lib.MegiddoGamepad;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StorageSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public abstract class Default extends CommandBaseOpMode {
    protected DriveSubsystem driveSubsystem;
    protected ShooterSubsystem shooter;
    protected IntakeSubsystem intake;
    protected StorageSubSystem storage;
    protected VisionSubsystem vision;
    MegiddoGamepad Gamepad1 = new MegiddoGamepad();
    MegiddoGamepad Gamepad2 = new MegiddoGamepad();
    @Override
    public void init() {
        Robot.init(this);
        shooter = new ShooterSubsystem();
        driveSubsystem = new DriveSubsystem();
        intake = new IntakeSubsystem();
        storage = new StorageSubSystem();
        vision = new VisionSubsystem();

        driveSubsystem.setHorizontalSpeed(0.6);
        driveSubsystem.setAnglerSpeed(0.6);
        driveSubsystem.setVerticalSpeed(0.6);
    }

    @Override
    public void update() {
        super.update();
        Gamepad1.update(gamepad1);
        Gamepad2.update(gamepad2);
    }
}
