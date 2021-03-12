package org.firstinspires.ftc.teamcode.lib;

import android.os.Build;

import java.util.List;
import java.util.function.BooleanSupplier;

/*
public class Button implements AutoCloseable  {
    private final BooleanSupplier supplier;
    private boolean last;
    static List<Button> buttons;

    public Button(BooleanSupplier supplier) {
        this.supplier = supplier;
        buttons.add(this);
    }

    @Override
    public void close() throws Exception {
        buttons.remove(this);
    }

    public boolean isPressed() {
        return supplier.getAsBoolean();
    }

    public boolean getLast() {
        return last;
    }

    public boolean get() {
        return supplier.getAsBoolean();
    }

    private void updateCurrent() {
        last = get();
    }

    public static void update() {
        for (Button button: buttons) {
            button.updateCurrent();
        }
    }
}
*/