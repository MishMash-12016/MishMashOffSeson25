package org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Servo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleCrServo;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleRevHub;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;

import java.util.ArrayList;

/**
 * A subsystem for managing one or more continuous rotation servos (CRServos).
 * Wraps {@link CuttleCrServo} instances to simplify control and command creation.
 */
public class CrServoSubsystem extends SubsystemBase {

    ArrayList<CuttleCrServo> servoList = new ArrayList<>();

    /**
     * Constructs a subsystem using a {@link CuttleRevHub}-based CRServo.
     *
     * @param revHub         the Rev Hub the CRServo is connected to
     * @param servoPort      the port on the hub
     * @param servoDirection the logical direction of the CRServo
     */
    public CrServoSubsystem(CuttleRevHub revHub, int servoPort, Direction servoDirection) {
        CuttleCrServo servo = new CuttleCrServo(revHub, servoPort).setDirection(servoDirection);
        servoList.add(servo);
    }

    /**
     * Constructs a subsystem using a {@link HardwareMap}-based CRServo.
     *
     * @param hardwareMap    the hardware map from the OpMode
     * @param servoName      the name of the CRServo in the configuration
     * @param servoDirection the logical direction of the CRServo
     */
    public CrServoSubsystem(HardwareMap hardwareMap, String servoName, Direction servoDirection) {
        CuttleCrServo servo = new CuttleCrServo(hardwareMap, servoName).setDirection(servoDirection);
        servoList.add(servo);
    }

    /**
     * Constructs a subsystem from a preconfigured {@link CuttleCrServo}.
     *
     * @param servo the CRServo to manage
     */
    public CrServoSubsystem(CuttleCrServo servo){
        servoList.add(servo);
    }

    /**
     * Creates a command that sets the power of all CRServos in the subsystem.
     *
     * @param power the power to set [-1.0, 1.0]
     * @return an {@link InstantCommand} that sets the power
     */
    public Command setPowerCommand(double power){
        return new InstantCommand(() -> setPower(power), this);
    }

    /**
     * Sets the power of all CRServos in the subsystem.
     *
     * @param power the power to set [-1.0, 1.0]
     */
    public void setPower(double power){
        for (CuttleCrServo servo : servoList) {
            servo.setPower(power);
        }
    }

    /**
     * Returns the current power of the first CRServo in the subsystem.
     * Assumes all servos are synchronized.
     *
     * @return the power level [-1.0, 1.0]
     */
    public double getPower(){
        return servoList.get(0).getPower();
    }

    /**
     * Adds another CRServo using a {@link CuttleRevHub} and returns this instance for chaining.
     *
     * @param revHub         the Rev Hub to use
     * @param servoPort      the servo port number
     * @param servoDirection the logical direction of the CRServo
     * @return this subsystem instance
     */
    public CrServoSubsystem withServo(CuttleRevHub revHub, int servoPort, Direction servoDirection) {
        CuttleCrServo servo = new CuttleCrServo(revHub, servoPort).setDirection(servoDirection);
        servoList.add(servo);
        return this;
    }

    /**
     * Adds another CRServo using {@link HardwareMap} and returns this instance for chaining.
     *
     * @param hardwareMap    the hardware map to use
     * @param servoName      the name of the CRServo
     * @param servoDirection the logical direction of the CRServo
     * @return this subsystem instance
     */
    public CrServoSubsystem withServo(HardwareMap hardwareMap, String servoName, Direction servoDirection) {
        CuttleCrServo servo = new CuttleCrServo(hardwareMap, servoName).setDirection(servoDirection);
        servoList.add(servo);
        return this;
    }

    /**
     * Adds an existing {@link CuttleCrServo} to the subsystem and returns this instance for chaining.
     *
     * @param servo the CRServo to add
     * @return this subsystem instance
     */
    public CrServoSubsystem withServo(CuttleCrServo servo){
        servoList.add(servo);
        return this;
    }
}
