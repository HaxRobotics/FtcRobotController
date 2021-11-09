package org.firstinspires.ftc.teamcode.teleopCode;

public enum WheelPositions {
    FRONT_LEFT(0),
    FRONT_RIGHT(1),
    BACK_LEFT(2),
    BACK_RIGHT(3);

    public final int value;

    WheelPositions(int value) {
        this.value = value;
    }
}
