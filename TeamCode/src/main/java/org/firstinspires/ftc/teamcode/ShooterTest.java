package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.lib.MegiddoGamepad;

@Disabled
@TeleOp(name = "Shooter Test", group="Iterative Opmode")
public class ShooterTest extends OpMode {

    private DcMotor right_motor;
    private DcMotor left_motor;
    private static double power = 1;
    private static double change_rate = 0.1;

    MegiddoGamepad Gamepad1;
    MegiddoGamepad Gamepad2;

    @Override
    public void init() {
        right_motor = hardwareMap.dcMotor.get("port0");
        left_motor = hardwareMap.dcMotor.get("port1");

        right_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        left_motor.setDirection(DcMotorSimple.Direction.FORWARD);

        Gamepad1 = new MegiddoGamepad();
        Gamepad2 = new MegiddoGamepad();
    }

    @Override
    public void loop() {
        Gamepad1.update(gamepad1);
        Gamepad2.update(gamepad2);

        if (Gamepad1.a_Pressed()) {
            if (right_motor.getPower() == 0) {
                right_motor.setPower(power);
                left_motor.setPower(power);
            } else {
                right_motor.setPower(0);
                left_motor.setPower(0);
            }
        } else if (right_motor.getPower() != 0) {
            right_motor.setPower(power);
            left_motor.setPower(power);
        }


        if (Gamepad1.dpad_up_Pressed()) {
            power += change_rate;
        } else if (Gamepad1.dpad_down_Pressed()) {
            power -= change_rate;
        }

        if (Gamepad1.dpad_left_Pressed()) {
            change_rate *= 2;
        } else if (Gamepad1.dpad_right_Pressed()) {
            change_rate /= 2;
        }
        telemetry.addData("pos", left_motor.getCurrentPosition());
        telemetry.addData("power", "%.3f", power);
        telemetry.addData("change rate", "%.3f", change_rate);
        telemetry.update();
    }
}
