package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.util.logging.SocketHandler;

public abstract class Default extends OpMode {
    DriveSubsystem driveSubsystem;
    ShooterSubsystem shooter;

    @Override
    public void init() {
//        shooter = new ShooterSubsystem(hardwareMap);
        driveSubsystem = new DriveSubsystem(hardwareMap);
    }
}
