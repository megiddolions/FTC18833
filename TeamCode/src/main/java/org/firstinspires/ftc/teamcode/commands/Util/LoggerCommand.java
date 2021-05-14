package org.firstinspires.ftc.teamcode.commands.Util;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.teamcode.lib.Util;

import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class LoggerCommand extends CommandBase {
    private final FileWriter log_file;
    private final LogThread log_thread;
    private final Thread thread;

    @SuppressLint("SdCardPath")
    public LoggerCommand(List<String> log_entries, List<Supplier<Object>> log_suppliers) {
        FileWriter log_file = null;
        LogThread log_thread = null;
        Thread thread = null;
        try {
            log_file = new FileWriter("/sdcard/FIRST/log/" + Util.getTime() + ".csv");
            for (String key : log_entries) {
                log_file.write(key + ",");
            }
            log_thread = new LogThread(log_file, log_suppliers);
            thread = new Thread(log_thread);
        } catch (IOException e) {
            e.printStackTrace();
        }
        this.log_file = log_file;
        this.log_thread = log_thread;
        this.thread = thread;
    }

    @Override
    public void initialize() {
        thread.start();
    }

    @Override
    public void end(boolean interrupted) {
        try {
            log_thread.stop();
            log_file.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    public boolean isFinished() {
        return log_file == null;
    }

    private static class LogThread implements Runnable {
        private final FileWriter log_file;
        private final List<Supplier<Object>> log_suppliers;
        private boolean running = true;

        private LogThread(FileWriter log_file, List<Supplier<Object>> log_suppliers) {
            this.log_file = log_file;
            this.log_suppliers = log_suppliers;
        }

        @Override
        public void run() {
            try {
                while (running) {
                    for (Supplier<Object> supplier : log_suppliers) {
                        log_file.write(supplier.get().toString() + ",");
                    }
                    log_file.write('\n');
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        public void stop() {
            running = false;
        }
    }
}
