package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Drive", group = "Iterative Opmode")
public class Drive extends Default {
    @Override
    public void loop() {
        Gamepad1.update(gamepad1);
        Gamepad2.update(gamepad2);

        if (gamepad1.dpad_left || gamepad1.left_bumper) {
            driveSubsystem.driveLeft(1);
        } else if (gamepad1.dpad_right || gamepad1.right_bumper) {
            driveSubsystem.driveRight(1);
        } else {
            driveSubsystem.setPower(-gamepad1.left_stick_y * 0.7, - gamepad1.right_stick_y * 0.7);
//            driveSubsystem.differentialDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
        }

        if (Gamepad2.left_bumper_Pressed()) {
            intake.toggle(1);
        } else if (Math.abs(gamepad2.right_stick_y) > 0.1) {
            intake.toggle(-gamepad2.right_stick_y - 0.01);
        } else if (shooter.getIndex() != 1) {
            intake.toggle(0);
        }

        if (Gamepad2.right_bumper_Pressed()){
            shooter.toggle_index(1);
        } else if (Math.abs(gamepad2.left_stick_y) > 0.1) {
            shooter.index(-gamepad2.left_stick_y - 0.01);
        } else if (shooter.getIndex() != 1) {
            shooter.index(0);
        }

        if (Gamepad2.a_Pressed()) {
            shooter.toggle(0.53);
        }

        if (Gamepad2.dpad_up_Pressed()) {
            shooter.setLift(shooter.getLift() + 0.05);
        } else if (Gamepad2.dpad_down_Pressed()) {
            shooter.setLift(shooter.getLift() - 0.05);
        }

        telemetry.addData("Lift", shooter.getLift());
        telemetry.update();
    }
}
