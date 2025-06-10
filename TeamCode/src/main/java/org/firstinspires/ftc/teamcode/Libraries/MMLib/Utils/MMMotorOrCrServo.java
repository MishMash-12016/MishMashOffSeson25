package org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleCrServo;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleMotor;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;

/**
 * A class that handles the interaction of motors and crServos as a single object
 */
public class MMMotorOrCrServo {

    private CuttleMotor motor;
    private CuttleCrServo crServo;

    public MMMotorOrCrServo(CuttleMotor motor) {
        this.motor = motor;
    }

    public MMMotorOrCrServo(CuttleCrServo crServo) {
        this.crServo = crServo;
    }

    /**
     * sets the motor/crServo power
     *
     * @param power motor power (-1.0 to 1.0)
     */
    public void setPower(double power) {
        if (motor != null) {
            motor.setPower(power);
        } else if (crServo != null) {
            crServo.setPower(power);
        }
    }

    public void setDirection(Direction direction) {
        if (motor != null) {
            motor.setDirection(direction);
        } else if (crServo != null) {
            crServo.setDirection(direction);
        }
    }

    /**
     * sets the zero power behavior(!! this only effect motors !!)
     */
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        if (motor != null) {
            motor.setZeroPowerBehaviour(zeroPowerBehavior);
        }
    }

    public double getPower() {
        if (motor != null) {
            return motor.getPower();
        } else {
            return crServo.getPower();
        }
    }
}
