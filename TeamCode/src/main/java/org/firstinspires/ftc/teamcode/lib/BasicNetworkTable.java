package org.firstinspires.ftc.teamcode.lib;

import java.io.DataInputStream;
import java.util.Map;

public class BasicNetworkTable implements Runnable {
    private DataInputStream in;
    public Map<String, Double> vars;

    public BasicNetworkTable(DataInputStream in) {
        this.in = in;
    }

    @Override
    public void run() {

    }
}
