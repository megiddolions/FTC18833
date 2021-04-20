package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.lib.MegiddoGamepad;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.openftc.easyopencv.OpenCvCameraException;

import java.util.logging.SocketHandler;

public abstract class Default extends OpMode {
    protected DriveSubsystem driveSubsystem;
    protected ShooterSubsystem shooter;
    protected IntakeSubsystem intake;
    MegiddoGamepad Gamepad1 = new MegiddoGamepad();
    MegiddoGamepad Gamepad2 = new MegiddoGamepad();
    @Override
    public void init() {
        Robot.getInstance().init(this);
        shooter = new ShooterSubsystem(hardwareMap);
        driveSubsystem = new DriveSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
    }
}
