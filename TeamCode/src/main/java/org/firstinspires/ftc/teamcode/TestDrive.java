package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp(name="DriveTest")
public class TestDrive extends OpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor rearLeft;
    private DcMotor rearRight;

    @Override
    public void init() {
        rearLeft = hardwareMap.dcMotor.get("RearLeft");
        rearRight = hardwareMap.dcMotor.get( "RearRight");
        frontLeft = hardwareMap.dcMotor.get("FrontLeft");
        frontRight = hardwareMap.dcMotor.get("FrontRight");

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {
        frontLeft.setPower(-gamepad1.left_stick_y);
        rearLeft.setPower(-gamepad1.left_stick_y);
        frontRight.setPower(-gamepad1.right_stick_y);
        rearRight.setPower(-gamepad1.right_stick_y);

        telemetry.update();
    }

    public void setMode(DcMotor.RunMode mode) {
        rearLeft.setMode(mode);
        rearRight.setMode(mode);
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
    }
    public int getLeftTicks() {
        return -rearRight.getCurrentPosition();
    }
    public int getRightTicks() {
        return -frontLeft.getCurrentPosition();
    }
    public int getCenterTicks() {
        return frontRight.getCurrentPosition();
    }


}
