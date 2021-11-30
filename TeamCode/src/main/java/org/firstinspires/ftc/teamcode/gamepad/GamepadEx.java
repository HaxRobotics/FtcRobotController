package org.firstinspires.ftc.teamcode.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.gamepad.GamepadKeys.Button;
import org.firstinspires.ftc.teamcode.gamepad.GamepadKeys.Trigger;

import java.util.EnumMap;

// big boi gamepad
public class GamepadEx {
    // gamepad we are building off of
    public Gamepad gamepad;

    // maps each button to a button reader
    private final EnumMap<Button, ButtonReader> buttonReaders;
    // construct new GamepadEx
    public GamepadEx(Gamepad gamepad) {
        this.gamepad= gamepad;
        buttonReaders = new EnumMap<>(Button.class);
        // assign each button a button reader
        for (Button button: Button.values()) {
            buttonReaders.put(button, new ButtonReader(this, button));
        }
    }
    // return button value for given button
    public boolean getButton(Button button) {
        boolean buttonValue;
        // no better way to do this, sadly, unless I want to do some dumb shit with looping through a class's members
        switch (button) {
            case Y:
                buttonValue = gamepad.y;
                break;
            case X:
                buttonValue = gamepad.x;
                break;
            case A:
                buttonValue = gamepad.a;
                break;
            case B:
                buttonValue = gamepad.b;
                break;
            case LEFT_BUMPER:
                buttonValue = gamepad.left_bumper;
                break;
            case RIGHT_BUMPER:
                buttonValue = gamepad.right_bumper;
                break;
            case BACK:
                buttonValue = gamepad.back;
                break;
            case START:
                buttonValue = gamepad.start;
                break;
            case DPAD_UP:
                buttonValue = gamepad.dpad_up;
                break;
            case DPAD_DOWN:
                buttonValue = gamepad.dpad_down;
                break;
            case DPAD_LEFT:
                buttonValue = gamepad.dpad_left;
                break;
            case DPAD_RIGHT:
                buttonValue = gamepad.dpad_right;
                break;
            case LEFT_STICK_BUTTON:
                buttonValue = gamepad.left_stick_button;
                break;
            case RIGHT_STICK_BUTTON:
                buttonValue = gamepad.right_stick_button;
                break;
            default:
                buttonValue = false;
                break;
        }
        return buttonValue;
    }
    // returns trigger value for given trigger
    public double getTrigger(Trigger trigger) {
        double triggerValue;
        switch (trigger) {
            case LEFT_TRIGGER:
                triggerValue = gamepad.left_trigger;
                break;
            case RIGHT_TRIGGER:
                triggerValue = gamepad.right_trigger;
                break;
            default:
                triggerValue = 0;
                break;
        }
        return triggerValue;
    }
    // return the y value for the left stick
    // negative taken bc left is positive and right is negative
    public double getLeftY() {
        return -gamepad.left_stick_y;
    }
    // return the y value for the right stick
    // negative taken bc left is positive and right is negative
    public double getRightY() {
        return -gamepad.right_stick_y;
    }
    // return the x value for the left stick
    public double getLeftX() {
        return gamepad.left_stick_x;
    }
    // return the x value for the right stick
    public double getRightX() {
        return gamepad.right_stick_x;
    }
    // checks if the given button was just pressed
    public boolean wasJustPressed(Button button) {
        return buttonReaders.get(button).wasJustPressed();
    }
    // checks if the given button was just released
    public boolean wasJustReleased(Button button) {
        return buttonReaders.get(button).wasJustReleased();
    }
    // updates button values
    public void readButtons() {
        for (ButtonReader reader : buttonReaders.values()) {
            reader.readValue();
        }
    }
    // checks if the button is down
    public boolean isDown(Button button) {
        return buttonReaders.get(button).isDown();
    }
    // checks if the button's state has just changed
    public boolean stateJustChanged(Button button) {
        return buttonReaders.get(button).stateJustChanged();
    }
    // check if button is present in hash map, avoids null pointer exceptions
    private boolean validButton(Button button) {
        return buttonReaders.containsKey(button);
    }
}
