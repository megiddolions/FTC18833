package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants.IndexConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.commandftc.RobotUniversal.hardwareMap;

public class StorageSubSystem extends SubsystemBase {
    private final DcMotor indexer;
    private final ColorSensor colorSensor;

//    private final DistanceSensor distanceSensor;

    private final ColorRange ringColor = new ColorRange()
            .red(550, 3000).green(0, 2000).blue(0, 2000);

    public StorageSubSystem() {
        indexer = hardwareMap.dcMotor.get("IndexMotor");
        colorSensor = hardwareMap.get(ColorSensor.class, "StorageSensor");
        indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexer.setTargetPosition(0);
        indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void set_for_autonomous() {
        indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexer.setPower(1);
    }

    public void set_for_commands() {
        indexer.setPower(0);
        indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void index(double power) {
        indexer.setPower(power);
    }

    public double getIndex() {
        return indexer.getPower();
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

    public void index_distance(double mm) {
//        indexer.setTargetPosition((int)(indexer.getCurrentPosition() + IndexConstants.mm_to_ticks.apply(125)));
        indexer.setTargetPosition((int)(indexer.getCurrentPosition() + IndexConstants.mm_to_ticks.apply(mm)));
    }

    public boolean isBusy() {
        return indexer.isBusy();
    }
    
    public int getEncoder() {
        return indexer.getCurrentPosition(); 
    }
}
