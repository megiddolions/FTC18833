package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.lib.MegiddoGamepad;
import org.firstinspires.ftc.teamcode.lib.Util;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Constants.NetworkConstants;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.Socket;
import java.nio.ByteBuffer;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Velocity Test", group="Iterative Opmode")
public class VelocityControl extends OpMode implements Runnable {

    private static int option = 0;
    private static double velocity = 10 * 60;
    private static double change_rate = 125;
    // Subsystems
    private ShooterSubsystem shooter;
    // Gamepads
    MegiddoGamepad Gamepad1;
    MegiddoGamepad Gamepad2;
    // Debug
    boolean active = false;
    boolean shooter_active;

    @Override
    public void init() {
        shooter = new ShooterSubsystem(hardwareMap);

        Gamepad1 = new MegiddoGamepad();
        Gamepad2 = new MegiddoGamepad();

        active = true;
        Thread thread = new Thread(this);
        thread.start();

        shooter_active = false;
    }

    @Override
    public void stop() {
        active = false;
    }

    @Override
    public void loop() {
        Gamepad1.update(gamepad1);
        Gamepad2.update(gamepad2);

        if (shooter_active) {
            shooter.setVelocity(velocity);
        }

        if (Gamepad1.a_Pressed()) {
            shooter_active = !shooter_active;
            if (!shooter_active)
                shooter.setVelocity(0);
        }


        if (Gamepad1.dpad_down_Pressed()) {
            option++;
            if (option == 6)
                option = 0;
        } else if (Gamepad1.dpad_up_Pressed()) {
            option--;
            if (option == -1)
                option = 5;
        }

        if (Gamepad1.x_Pressed()) {
            shooter.toggle_index(1);
        }

        // UI
        if (Gamepad1.dpad_left_Pressed()) {
            switch (option) {
                case 0:
                    velocity += change_rate;
                    break;
                case 1:
                    shooter.pid.p += change_rate;
                    break;
                case 2:
                    shooter.pid.i += change_rate;
                    break;
                case 3:
                    shooter.pid.d += change_rate;
                    break;
                case 4:
                    shooter.pid.f += change_rate;
                    break;
                case 5:
                    change_rate /= 2;
                    break;
            }
        } else if (Gamepad1.dpad_right_Pressed()) {
            switch (option) {
                case 0:
                    velocity -= change_rate;
                    break;
                case 1:
                    shooter.pid.p -= change_rate;
                    break;
                case 2:
                    shooter.pid.i -= change_rate;
                    break;
                case 3:
                    shooter.pid.d -= change_rate;
                    break;
                case 4:
                    shooter.pid.f -= change_rate;
                    break;
                case 5:
                    change_rate /= 2;
                    break;
            }
        }

//        telemetry.addData("time", getRuntime());
        telemetry.addData((option == 0 ? "* " : "   ") + "Velocity(RPM)", "%.3f", velocity);
        telemetry.addData((option == 1 ? "* " : "   ") + "P", shooter.pid.p);
        telemetry.addData((option == 2 ? "* " : "   ") + "I", shooter.pid.i);
        telemetry.addData((option == 3 ? "* " : "   ") + "D", shooter.pid.d);
        telemetry.addData((option == 4 ? "* " : "   ") + "F", shooter.pid.f);
        telemetry.addData((option == 5 ? "* " : "   ") + "change rate", "%.3f", change_rate);
        telemetry.update();
    }

    @Override
    public void run() {
        Socket server;
        DataOutputStream out;
        DataInputStream in;
        // not including time var
        int vars_count = 2;
        try {
            server = new Socket(NetworkConstants.computer_ip, NetworkConstants.server_port);
            out = new DataOutputStream(server.getOutputStream());
//            in = new DataInputStream(server.getInputStream());

            out.write(ByteBuffer.allocate(4).putInt(vars_count).array());

            while (active) {
                out.write(ByteBuffer.allocate(8 + 8 * 2)
                        .putDouble(getRuntime())
                        .putDouble(shooter.getLeftVelocity())
                        .putDouble(shooter.getRightVelocity())
                        .array());
                TimeUnit.MILLISECONDS.sleep(100);
            }


            server.close();
        } catch (IOException | InterruptedException e) {
            System.out.println(e.toString());
            telemetry.addData("error", e.toString());
            telemetry.update();
        }
    }
}