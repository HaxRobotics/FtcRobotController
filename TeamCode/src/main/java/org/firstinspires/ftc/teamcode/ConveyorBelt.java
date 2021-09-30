package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ConveyorBelt {
    // Use to create conveyorBelt and interact with it. Key search for ER1 for possible errors.
    private final DcMotor upperMotor;
    private final DcMotor lowerMotor;
    // May need to add power attribute if motors need to run at different speeds. ER1

    // Init. Starts with power of 0. Sets up both motors with the names provided.
    public ConveyorBelt(HardwareMap hardwareMap, String lowerMotorName, String upperMotorName) {
        // Takes in motor names and creates the motor to run.
        // All power comes from lower power, read above in attribute declaration about possible error.
        lowerMotor = hardwareMap.get(DcMotor.class, lowerMotorName);
        upperMotor = hardwareMap.get(DcMotor.class, upperMotorName);
        // Sets power to 0 upon initialization
        upperMotor.setPower(0);
        lowerMotor.setPower(0);
    }

    // Returns the current power of the robot as a double. For actual math.
    public double currentPower() {
        return lowerMotor.getPower();
    }

    // Returns the string of the power, used for telemetry.
    public String toString() {
        return String.valueOf(lowerMotor.getPower());
    }

    // Used to set the power of the motors in the belt. ER1
    public double setPower(double power) {
        // Power can be negetive. ER1
        // See if the motors need diffrent powers. Possible physical masifestation. ER1
        lowerMotor.setPower(power);
        upperMotor.setPower(power);
        // Returns power if want to use after calling
        return power;
    }
}
