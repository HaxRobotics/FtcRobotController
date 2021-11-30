package org.firstinspires.ftc.teamcode.gamepad;

import java.util.function.BooleanSupplier;
// specialty class for reading buttons that will be used as toggles
public class ToggleButtonReader extends ButtonReader {

    private boolean currToggleState;

    public ToggleButtonReader(GamepadEx gamepad, GamepadKeys.Button button) {
        super(gamepad, button);

        currToggleState = false;
    }

    public ToggleButtonReader(BooleanSupplier buttonValue) {
        super(buttonValue);

        currToggleState = false;
    }

    // return the current state of the toggler
    public boolean getState() {
        // toggles on press of button
        if (wasJustPressed()) {
            currToggleState = !currToggleState;
        }
        return currToggleState;
    }
}
