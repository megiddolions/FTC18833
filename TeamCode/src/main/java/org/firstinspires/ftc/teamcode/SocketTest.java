package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.ByteBuffer;
import java.text.RuleBasedCollator;
import java.util.concurrent.TimeUnit;
import java.util.stream.Stream;

@TeleOp(name = "Socket Test", group="Iterative Opmode")
public class SocketTest extends OpMode implements Runnable {
    boolean active = true;
    int count = 0;

    @Override
    public void init() {
        active = true;
        Thread thread = new Thread(this);
        thread.start();
        count = 0;
    }

    @Override
    public void loop() {
        count++;
    }

    @Override
    public void stop() {
        active = false;
    }

    public void run() {
        byte[] bin = new byte[12];
        Socket server;
        DataOutputStream out;
        DataInputStream in;
        try {
            server = new Socket("192.168.49.193", 5038);
            out = new DataOutputStream(server.getOutputStream());
            in = new DataInputStream(server.getInputStream());

            while (active) {

                ByteBuffer.wrap(bin, 0, 8).putLong((long)(getRuntime() * 1000));
                ByteBuffer.wrap(bin, 8, 4).putInt(count);

                out.write(bin);
                telemetry.addData("bin", bin.length);
                telemetry.addData("count", count);
                telemetry.addData("time", getRuntime());
                telemetry.update();
                TimeUnit.MILLISECONDS.sleep(100);
            }


            server.close();


        } catch (IOException | InterruptedException e) {
            System.out.println(e.toString());
            telemetry.addData("error", e.toString());
            telemetry.update();
        }
    }

    private void send_int(DataOutputStream output, int v) {
        char[] s = Integer.toHexString(v).toCharArray();
        for (char c : s) {
            try {
                output.write(c);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
}
