package org.firstinspires.ftc.teamcode.gamepad;

public interface KeyReader {
    /** Reads key value*/
    void readValue();
    /**Checks if key is down*/
    boolean isDown();
    /** Checks if key was just pressed */
    boolean wasJustPressed();
    /** Checks if key was just released*/
    boolean wasJustReleased();
    /** Checks if key state has just changed */
    boolean stateJustChanged();
}
