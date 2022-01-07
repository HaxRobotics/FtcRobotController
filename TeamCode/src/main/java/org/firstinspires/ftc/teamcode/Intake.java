package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Object to control an intake mechanism
 */
public class Intake {
    private final CRServo leftServo;
    private final CRServo rightServo;

    /**
     * Construct new Intake
     *
     * @param hardwareMap The hardware map to the control hub
     * @param leftServoName name of left servo
     * @param rightServoName name of right servo
     */
    public Intake(HardwareMap hardwareMap, String leftServoName, String rightServoName) {
        leftServo = hardwareMap.get(CRServo.class, leftServoName);
        rightServo = hardwareMap.get(CRServo.class, rightServoName);
    }

    // Runs intake to pick up objects
    public void in(double power) {
        leftServo.setPower(-power);
        rightServo.setPower(power);
    }
    public void in() {
        in(1);
    }
    // Runs outtake to place down objects
    public void out(double power) {
        leftServo.setPower(power);
        rightServo.setPower(-power);
    }
    public void out() {
        out(1);
    }
    // Stops intake/outtake from running
    public void stop() {
        leftServo.setPower(0);
        rightServo.setPower(0);
    }

    // Checks whether or not the crservos are running
    public boolean isRunning() {
        return leftServo.getPower() != 0 && rightServo.getPower() != 0;
    }
}
