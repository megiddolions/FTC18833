package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.lib.CommandBaseOpMode;
import org.firstinspires.ftc.teamcode.lib.MegiddoGamepad;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public abstract class Default extends CommandBaseOpMode {
    protected DriveSubsystem driveSubsystem;
    protected ShooterSubsystem shooter;
    protected IntakeSubsystem intake;
    MegiddoGamepad Gamepad1 = new MegiddoGamepad();
    MegiddoGamepad Gamepad2 = new MegiddoGamepad();
    @Override
    public void init() {
        Robot.getInstance().init(this);
        shooter = new ShooterSubsystem();
        driveSubsystem = new DriveSubsystem();
        intake = new IntakeSubsystem();

        driveSubsystem.setHorizontalSpeed(0.8);
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
