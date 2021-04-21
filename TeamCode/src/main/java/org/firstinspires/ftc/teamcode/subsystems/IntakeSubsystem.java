package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    DcMotor intake_motor;

    public IntakeSubsystem() {
        intake_motor = Robot.OpMode().hardwareMap.dcMotor.get("IntakeMotor");
        intake_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake_motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void intake(double speed) {
        intake_motor.setPower(speed);
    }

    public void toggle(double speed) {
        if (intake_motor.getPower() == 0) {
            intake_motor.setPower(speed);
        } else {
            intake_motor.setPower(0);
        }
    }

    public void update_toggle(double speed) {
        if (intake_motor.getPower() != 0) {
            intake_motor.setPower(speed);
        }
    }
}
