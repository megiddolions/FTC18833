package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.Util;

@TeleOp(name="Drive", group = "Iterative Opmode")
public class Drive extends Default {
    boolean constStorage = true;
    boolean constIntake = false;
    double speedGear;
    @Override
    public void loop() {
        update();
        if (gamepad1.dpad_left || gamepad1.left_bumper) {
            driveSubsystem.driveLeft(1);
        } else if (gamepad1.dpad_right || gamepad1.right_bumper) {
            driveSubsystem.driveRight(1);
        }else if((gamepad1.left_stick_button
                && Math.abs(gamepad1.left_stick_y) < 0.2)
                ||(gamepad1.right_stick_button && Math.abs(gamepad1.right_stick_y) < 0.2)){
            driveSubsystem.setPower(Util.maxAbs(-gamepad1.left_stick_y,-gamepad1.right_stick_y),
                    Util.maxAbs(-gamepad1.left_stick_y,-gamepad1.right_stick_y));
        } else {
            speedGear = (gamepad2.left_trigger > 0.1) ? 0.3 : ((gamepad2.right_trigger > 0.1) ? 1 : 0.7);
            driveSubsystem.setPower(-gamepad1.left_stick_y * speedGear, -gamepad1.right_stick_y * speedGear);
//            driveSubsystem.differentialDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
        }

//
//        if (Math.abs(gamepad2.right_stick_y) > 0.1) {
//            intake.intake(-gamepad2.right_stick_y);
//        } else {
//            intake.intake(0);
//        }

        constIntake = (0.9 < -gamepad2.left_stick_y && gamepad2.left_stick_button) || constIntake;
        constIntake = (!(-0.5 > -gamepad2.left_stick_y)) && constIntake;

        if (Math.abs(gamepad2.left_stick_y) > 0.1) {

            intake.intake((constIntake) ? 1 : -gamepad2.left_stick_y);
        } else if (!constIntake){
            intake.intake(0);
        }


        constStorage = (0.9 < -gamepad2.right_stick_y && gamepad2.right_stick_button) || constStorage;
        constStorage = (!(-0.5 > -gamepad2.right_stick_y)) && constStorage;
        if (Math.abs(gamepad2.right_stick_y) > 0.1) {

            storage.index((constStorage) ? 1 : -gamepad2.right_stick_y);
        } else if (!constStorage){
            storage.index(0);
            constStorage = true;
        }



        if (Gamepad2.y_Pressed()) {
            shooter.toggle(0.53);
        }

        if (Gamepad2.dpad_up_Pressed()) {
            shooter.setLift(shooter.getLift() + 0.05);
        } else if (Gamepad2.dpad_down_Pressed()) {
            shooter.setLift(shooter.getLift() - 0.05);
        }

//        telemetry.addData("Subsystems", Robot.subsystems.keySet());
//        telemetry.addData("Lift", shooter.getLift());
        telemetry.addData("constIntake:",constIntake);
        telemetry.addData("constStorage:",constStorage);
        telemetry.addData("Right Y:",gamepad2.right_stick_y);
        telemetry.addData("Left Y:",gamepad2.left_stick_y);

        telemetry.update();
    }
}
