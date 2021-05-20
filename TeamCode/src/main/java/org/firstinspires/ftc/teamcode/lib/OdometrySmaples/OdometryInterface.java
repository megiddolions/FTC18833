package org.firstinspires.ftc.teamcode.lib.OdometrySmaples;

public interface OdometryInterface {
    double getX();
    double getY();
    double getA();
    void update(double left, double right, double center);
}
