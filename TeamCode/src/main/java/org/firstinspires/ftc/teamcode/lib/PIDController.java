package org.firstinspires.ftc.teamcode.lib;

import static org.commandftc.RobotUniversal.*;

public class PIDController {

    private double kp;
    private double ki;
    private double kd;

    private double time;


    private double last_error = 0;
    private double error = 0;
    private double k_error = 0;
    private double i_error = 0;
    private double d_error = 0;


    /**
     * set PID gain and time between loop updates in milliseconds
     */
    public PIDController(double Kp, double Ki, double Kd) {
        kp = Kp;
        ki = Ki;
        kd = Kd;
        time = opMode.getRuntime();
//        var a = 0;
    }


    public double output(double setPoint, double currentState) {
        double last_time = time;
        time = opMode.getRuntime();
        double dt = (time - last_time);

        // calculate the error term
        error = setPoint - currentState;
        // obtain the proportional error (total waste of memory lul)
        k_error = error;
        d_error = (error - last_error) / dt;
        i_error = i_error + (error * dt);
        last_error = error;

        return (k_error * kp) + (i_error * ki) + (d_error * kd);
    }

    public void clear() {
        last_error = 0;
        error = 0;
        k_error = 0;
        d_error = 0;
        i_error = 0;
    }
}
