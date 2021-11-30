package org.firstinspires.ftc.teamcode.gamepad;

import java.util.function.BooleanSupplier;

// Reads value of Button states
public class ButtonReader implements KeyReader {
    // last state of the button
    private boolean lastState;
    // current state of the button
    private boolean currState;
    // state of the button
    private final BooleanSupplier buttonState;

    // Constructor that uses a gamepad and button to read
    public ButtonReader(GamepadEx gamepad, GamepadKeys.Button button) {
        buttonState = () -> gamepad.getButton(button);
        currState = buttonState.getAsBoolean();
        lastState = currState;
    }
    // Constructor that takes in function for button value, initializes states
    // for use without a gamepad
    public ButtonReader(BooleanSupplier buttonValue) {
        buttonState = buttonValue;
        currState = buttonState.getAsBoolean();
        lastState = currState;
    }

    /**
     * Reads key value
     */
    public void readValue() {
        lastState = currState;
        currState = buttonState.getAsBoolean();
    }

    /**
     * Checks if key is down
     */
    public boolean isDown() {
        return buttonState.getAsBoolean();
    }

    /**
     * Checks if key was just pressed
     */
    public boolean wasJustPressed() {
        return !lastState && currState;
    }

    /**
     * Checks if key was just released
     */
    public boolean wasJustReleased() {
        return lastState && !currState;
    }

    /**
     * Checks if key state has just changed
     */
    public boolean stateJustChanged() {
        return lastState != currState;
    }
}
