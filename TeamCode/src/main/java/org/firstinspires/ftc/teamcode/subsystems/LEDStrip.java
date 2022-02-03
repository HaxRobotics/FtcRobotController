package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LEDStrip {
    // patterns for alliances
    private final RevBlinkinLedDriver.BlinkinPattern[] allianceColors = {
            RevBlinkinLedDriver.BlinkinPattern.RED,
            RevBlinkinLedDriver.BlinkinPattern.BLUE
    };
    // blinkin led driver
    // COLOR 1 is set as RED and COLOR 2 is set as BLUE
    private final RevBlinkinLedDriver blinkin;
    public Alliance alliance;
    // patterns for where the shipping element is detected
    private RevBlinkinLedDriver.BlinkinPattern[] locationColors;

    public LEDStrip(@NonNull HardwareMap hw, String driverName, Alliance alliance) {
        blinkin = hw.get(RevBlinkinLedDriver.class, driverName);
        this.alliance = alliance;
        locationColors = new RevBlinkinLedDriver.BlinkinPattern[]{
                RevBlinkinLedDriver.BlinkinPattern.RED,
                RevBlinkinLedDriver.BlinkinPattern.GREEN,
                RevBlinkinLedDriver.BlinkinPattern.BLUE,
                RevBlinkinLedDriver.BlinkinPattern.ORANGE
        };
    }

    public void setLocationColors(RevBlinkinLedDriver.BlinkinPattern[] colors) {
        locationColors = colors;
    }

    public void allianceSolid() {
        blinkin.setPattern(allianceColors[alliance.val]);
    }

    public void allianceBlink() {
        if (alliance == Alliance.RED) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_SLOW);
        } else {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_SLOW);
        }
    }

    public void detected(@NonNull ShippingElementDetector.ElementLocation location) {
        blinkin.setPattern(locationColors[location.val]);
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        blinkin.setPattern(pattern);
    }

    public enum Alliance {
        RED(0),
        BLUE(1);

        int val;

        Alliance(int i) {
            val = i;
        }

        public int val() {
            return val;
        }
    }
}
