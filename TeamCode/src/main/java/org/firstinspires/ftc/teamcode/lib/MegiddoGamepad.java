package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * This class was made by python scripts in IPython 7.15.0
 */
public class MegiddoGamepad {

    public boolean dpad_up = false;
    private boolean _dpad_up = false;
    public boolean dpad_down = false;
    private boolean _dpad_down = false;
    public boolean dpad_left = false;
    private boolean _dpad_left = false;
    public boolean dpad_right = false;
    private boolean _dpad_right = false;
    public boolean a = false;
    private boolean _a = false;
    public boolean b = false;
    private boolean _b = false;
    public boolean x = false;
    private boolean _x = false;
    public boolean y = false;
    private boolean _y = false;
    public boolean guide = false;
    private boolean _guide = false;
    public boolean start = false;
    private boolean _start = false;
    public boolean back = false;
    private boolean _back = false;
    public boolean left_bumper = false;
    private boolean _left_bumper = false;
    public boolean right_bumper = false;
    private boolean _right_bumper = false;
    public boolean left_stick_button = false;
    private boolean _left_stick_button = false;
    public boolean right_stick_button = false;
    private boolean _right_stick_button = false;

    public void update(Gamepad gamepad) {
        _dpad_up = dpad_up;
        dpad_up = gamepad.dpad_up;
        _dpad_down = dpad_down;
        dpad_down = gamepad.dpad_down;
        _dpad_left = dpad_left;
        dpad_left = gamepad.dpad_left;
        _dpad_right = dpad_right;
        dpad_right = gamepad.dpad_right;
        _a = a;
        a = gamepad.a;
        _b = b;
        b = gamepad.b;
        _x = x;
        x = gamepad.x;
        _y = y;
        y = gamepad.y;
        _guide = guide;
        guide = gamepad.guide;
        _start = start;
        start = gamepad.start;
        _back = back;
        back = gamepad.back;
        _left_bumper = left_bumper;
        left_bumper = gamepad.left_bumper;
        _right_bumper = right_bumper;
        right_bumper = gamepad.right_bumper;
        _left_stick_button = left_stick_button;
        left_stick_button = gamepad.left_stick_button;
        _right_stick_button = right_stick_button;
        right_stick_button = gamepad.right_stick_button;
    }

    public boolean dpad_up_Pressed () {
        return dpad_up && ! _dpad_up;
    }

    public boolean dpad_up_Held () {
        return dpad_up && _dpad_up;
    }

    public boolean dpad_up_Released () {
        return !dpad_up && _dpad_up;
    }

    public boolean dpad_down_Pressed () {
        return dpad_down && ! _dpad_down;
    }

    public boolean dpad_down_Held () {
        return dpad_down && _dpad_down;
    }

    public boolean dpad_down_Released () {
        return !dpad_down && _dpad_down;
    }

    public boolean dpad_left_Pressed () {
        return dpad_left && ! _dpad_left;
    }

    public boolean dpad_left_Held () {
        return dpad_left && _dpad_left;
    }

    public boolean dpad_left_Released () {
        return !dpad_left && _dpad_left;
    }

    public boolean dpad_right_Pressed () {
        return dpad_right && ! _dpad_right;
    }

    public boolean dpad_right_Held () {
        return dpad_right && _dpad_right;
    }

    public boolean dpad_right_Released () {
        return !dpad_right && _dpad_right;
    }

    public boolean a_Pressed () {
        return a && ! _a;
    }

    public boolean a_Held () {
        return a && _a;
    }

    public boolean a_Released () {
        return !a && _a;
    }

    public boolean b_Pressed () {
        return b && ! _b;
    }

    public boolean b_Held () {
        return b && _b;
    }

    public boolean b_Released () {
        return !b && _b;
    }

    public boolean x_Pressed () {
        return x && ! _x;
    }

    public boolean x_Held () {
        return x && _x;
    }

    public boolean x_Released () {
        return !x && _x;
    }

    public boolean y_Pressed () {
        return y && ! _y;
    }

    public boolean y_Held () {
        return y && _y;
    }

    public boolean y_Released () {
        return !y && _y;
    }

    public boolean guide_Pressed () {
        return guide && ! _guide;
    }

    public boolean guide_Held () {
        return guide && _guide;
    }

    public boolean guide_Released () {
        return !guide && _guide;
    }

    public boolean start_Pressed () {
        return start && ! _start;
    }

    public boolean start_Held () {
        return start && _start;
    }

    public boolean start_Released () {
        return !start && _start;
    }

    public boolean back_Pressed () {
        return back && ! _back;
    }

    public boolean back_Held () {
        return back && _back;
    }

    public boolean back_Released () {
        return !back && _back;
    }

    public boolean left_bumper_Pressed () {
        return left_bumper && ! _left_bumper;
    }

    public boolean left_bumper_Held () {
        return left_bumper && _left_bumper;
    }

    public boolean left_bumper_Released () {
        return !left_bumper && _left_bumper;
    }

    public boolean right_bumper_Pressed () {
        return right_bumper && ! _right_bumper;
    }

    public boolean right_bumper_Held () {
        return right_bumper && _right_bumper;
    }

    public boolean right_bumper_Released () {
        return !right_bumper && _right_bumper;
    }

    public boolean left_stick_button_Pressed () {
        return left_stick_button && ! _left_stick_button;
    }

    public boolean left_stick_button_Held () {
        return left_stick_button && _left_stick_button;
    }

    public boolean left_stick_button_Released () {
        return !left_stick_button && _left_stick_button;
    }

    public boolean right_stick_button_Pressed () {
        return right_stick_button && ! _right_stick_button;
    }

    public boolean right_stick_button_Held () {
        return right_stick_button && _right_stick_button;
    }

    public boolean right_stick_button_Released () {
        return !right_stick_button && _right_stick_button;
    }
}
