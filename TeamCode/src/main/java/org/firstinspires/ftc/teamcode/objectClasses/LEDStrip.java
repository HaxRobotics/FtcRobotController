package org.firstinspires.ftc.teamcode.objectClasses;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ShippingElementDetector;

public class LEDStrip {
    public enum Alliance {
        RED(0),
        BLUE(1);

        public int val;
        Alliance(int i) {
            val = i;
        }
    }
    // patterns for alliances
    private final RevBlinkinLedDriver.BlinkinPattern[] allianceColors = {
            RevBlinkinLedDriver.BlinkinPattern.RED,
            RevBlinkinLedDriver.BlinkinPattern.BLUE
    };
    // patterns for where the shipping element is detected
    private RevBlinkinLedDriver.BlinkinPattern[] locationColors;
    // blinkin led driver
    // COLOR 1 is set as RED and COLOR 2 is set as BLUE
    private final RevBlinkinLedDriver blinkin;
    public Alliance alliance;

    public LEDStrip(HardwareMap hw, String driverName, Alliance alliance) {
        blinkin = hw.get(RevBlinkinLedDriver.class, driverName);
        this.alliance = alliance;
        locationColors = new RevBlinkinLedDriver.BlinkinPattern[] {
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

    public void detected(ShippingElementDetector.ElementLocation location) {
        blinkin.setPattern(locationColors[location.val]);
    }
    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        blinkin.setPattern(pattern);
    }
}
