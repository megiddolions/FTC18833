package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.SubsystemBase;

public class StorageSubSystem extends SubsystemBase {
    private final DcMotor indexer;
    private final ColorSensor colorSensor;
//    private final DistanceSensor distanceSensor;
    private StorageState storageState = StorageState.Waiting;

    private enum StorageState {
        Waiting(false, true),
        Start(true, false),
        Middle(true, true),
        End(true, false);

        final boolean active;
        final boolean seen_ring;

        StorageState(boolean active, boolean seen_ring) {
            this.active = active;
            this.seen_ring = seen_ring;
        }
        
        StorageState next(boolean seeing_ring) {
            if (this.seen_ring == seeing_ring) {
                switch (this) {
                    case Waiting:
                        return Start;
                    case Start:
                        return Middle;
                    case Middle:
                        return End;
                    case End:
                        return Waiting;
                }
            }
            return this;
        }
    }

    private final ColorRange ringColor = new ColorRange()
            .red(550, 3000).green(0, 2000).blue(0, 2000);

    public StorageSubSystem() {
        indexer = Robot.opMode.hardwareMap.dcMotor.get("IndexMotor");
        colorSensor = Robot.opMode.hardwareMap.get(ColorSensor.class, "StorageSensor");
//        distanceSensor = Robot.opMode.hardwareMap.get(DistanceSensor.class, "StorageSensor");
        indexer.setDirection(DcMotorSimple.Direction.REVERSE);
        indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void index(double power) {
        indexer.setPower(power);
    }

    public double getIndex() {
        return indexer.getPower();
    }

    @Override
    public void periodic() {
        Robot.opMode.telemetry.addData("Red", colorSensor.red());
        Robot.opMode.telemetry.addData("Green", colorSensor.green());
        Robot.opMode.telemetry.addData("Blue", colorSensor.blue());
        Robot.opMode.telemetry.addData("has ring", seeing_ring());
//        Robot.opMode.telemetry.addData("Distance(mm)", distanceSensor.getDistance(DistanceUnit.MM));
        
        
        storageState = storageState.next(seeing_ring());

        if (storageState.active) {
            index(1);
        } else {
            index(0);
        }
    }

    private class ColorRange {
        public class Range {
            public double max;
            public double min;

            Range(double min, double max) {
                this.max = max;
                this.min = min;
            }

            public boolean is_valid(double value) {
                return value <= max && value >= min;
            }
        }

        public Range red = new Range(255, 0);
        public Range green = new Range(255, 0);
        public Range blue = new Range(255, 0);

        public ColorRange red(double min, double max) {
            red = new Range(min, max);
            return this;
        }
        public ColorRange green(double min, double max) {
            green = new Range(min, max);
            return this;
        }
        public ColorRange blue(double min, double max) {
            blue = new Range(min, max);
            return this;
        }

        public boolean in_range(ColorSensor sensor) {
            return red.is_valid(sensor.red()) && green.is_valid(sensor.green())
                    && blue.is_valid(sensor.blue());
        }

        public ColorRange() {

        }
    }

    public boolean seeing_ring() {
        //the red blue and green values we want
        return ringColor.in_range(colorSensor);
//        return distanceSensor.getDistance(DistanceUnit.MM) < 40;
    }

    private void storeRing(){
        //the movement amount of the motor for entering one ring
    }
}
