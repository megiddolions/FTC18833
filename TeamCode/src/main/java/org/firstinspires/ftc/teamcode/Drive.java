package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Drive", group = "Iterative Opmode")
public class Drive extends Default {
    @Override
    public void loop() {
        if (gamepad1.dpad_left || gamepad1.left_bumper) {
            driveSubsystem.driveLeft(1);
        } else if (gamepad1.dpad_right || gamepad1.right_bumper) {
            driveSubsystem.driveRight(1);
        } else {
            driveSubsystem.setPower(-gamepad1.left_stick_y, - gamepad1.right_stick_y);
        }
//        driveSubsystem.differentialDrive(-gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
    }
}
