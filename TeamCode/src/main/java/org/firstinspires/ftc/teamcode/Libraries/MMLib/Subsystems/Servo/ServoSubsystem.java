package org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Servo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleRevHub;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;

import java.util.ArrayList;
import java.util.Set;
import java.util.function.BooleanSupplier;

/**
 * A subsystem that wraps and manages one or more {@link CuttleServo} instances, allowing
 * for position control and command generation. Supports instant positioning,
 * gradual movement over time, and conditional positioning based on button input.
 */
//TODO: add manual logging using the subsystemName
public class ServoSubsystem extends SubsystemBase {

    ArrayList<CuttleServo> servoList = new ArrayList<>();
    private final String subsystemName;

    /**
     * Creates a ServoSubsystem using an existing {@link CuttleServo}.
     *
     * @param servo the servo instance to manage
     */
    public ServoSubsystem(CuttleServo servo, String subsystemName) {
        servoList.add(servo);
        this.subsystemName = subsystemName;
    }

    /**
     * Creates a ServoSubsystem using a {@link CuttleRevHub}-based servo configuration.
     *
     * @param revHub         the rev hub to which the servo is connected
     * @param servoPort      the port number on the hub
     * @param servoDirection the logical direction of the servo
     * @param offset         position offset for calibration
     */
    public ServoSubsystem(CuttleRevHub revHub, int servoPort,
                          Direction servoDirection, Double offset, String subsystemName) {
        this(new CuttleServo(revHub, servoPort).setOffset(offset)
                .setDirection(servoDirection), subsystemName);
    }

    /**
     * Creates a ServoSubsystem using a {@link HardwareMap}-based servo configuration.
     *
     * @param hardwareMap    the FTC hardware map
     * @param servoName      the name of the servo in the hardware configuration
     * @param servoDirection the logical direction of the servo
     * @param offset         position offset for calibration
     */
    public ServoSubsystem(HardwareMap hardwareMap, String servoName,
                          Direction servoDirection, Double offset, String subsystemName) {
        this(new CuttleServo(hardwareMap, servoName).setOffset(offset)
                .setDirection(servoDirection), subsystemName);
    }

    /**
     * Instantly moves all servos to the given position.
     *
     * @param position the target position [0.0, 1.0]
     * @return an InstantCommand that moves the servos immediately
     */
    public Command setPositionCommand(double position) {
        return new InstantCommand(() -> setPosition(position));
    }

    /**
     * Moves all servos to the target position gradually over a specified time.
     *
     * @param targetPose         the final target position [0.0, 1.0]
     * @param movementDurationMS the total movement duration in milliseconds
     * @return a Command that performs the motion over time
     */
    public Command moveToPositionOverTimeCommand(double targetPose, double movementDurationMS) {
        return new Command() {
            ElapsedTime currentTimeMS;
            double difference;
            double startPose;

            @Override
            public void initialize() {
                currentTimeMS = new ElapsedTime();
                currentTimeMS.reset();
                startPose = getPosition();
                difference = targetPose - startPose;
            }

            @Override
            public void execute() {
                setPosition(startPose + (currentTimeMS.milliseconds() / movementDurationMS) * difference);
            }

            @Override
            public boolean isFinished() {
                return currentTimeMS.milliseconds() >= movementDurationMS;
            }

            @Override
            public void end(boolean interrupted) {
                setPosition(targetPose);
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return Set.of(ServoSubsystem.this);
            }
        };
    }

    /**
     * Sets servo position based on a single button state.
     *
     * @param defaultPosition       the position to use when the button is not pressed
     * @param button                a BooleanSupplier providing the button state
     * @param buttonPressedPosition the position to move to when the button is pressed
     * @return a RunCommand that sets the position based on the button state
     */
    public Command setPositionByButton(double defaultPosition, BooleanSupplier button, double buttonPressedPosition) {
        return new RunCommand(() -> {
            if (button.getAsBoolean()) {
                setPosition(buttonPressedPosition);
            } else {
                setPosition(defaultPosition);
            }
        });
    }

    /**
     * Sets servo position based on the states of two buttons.
     *
     * @param defaultPosition the default position when neither button is pressed
     * @param button1         the first button
     * @param position1       the position for the first button
     * @param button2         the second button
     * @param position2       the position for the second button
     * @return a RunCommand that sets the position based on the button states
     */
    public Command setPositionByButton(double defaultPosition, BooleanSupplier button1, double position1, BooleanSupplier button2, double position2) {
        return new RunCommand(() -> {
            if (button1.getAsBoolean()) {
                setPosition(position1);
            } else if (button2.getAsBoolean()) {
                setPosition(position2);
            } else {
                setPosition(defaultPosition);
            }
        });
    }

    /**
     * Directly sets all attached servos to the specified position.
     *
     * @param position the target position [0.0, 1.0]
     */
    public void setPosition(double position) {
        for (CuttleServo servo : servoList) {
            servo.setPosition(position);
        }
    }

    /**
     * Gets the logical position of the first servo in the list,
     * taking into account offset and direction.
     *
     * @return the logical position [0.0, 1.0]
     */
    public double getPosition() {
        double noOffsetPose = servoList.get(0).getPosition() - servoList.get(0).getOffset();
        return servoList.get(0).getDirection() == Direction.REVERSE ?
                1 - noOffsetPose :
                noOffsetPose;
    }

    /**
     * Adds a preconfigured {@link CuttleServo} to the subsystem.
     *
     * @param servo the servo to add
     * @return this subsystem instance, for chaining
     */
    public ServoSubsystem withServo(CuttleServo servo) {
        servoList.add(servo);
        return this;
    }

    /**
     * Adds a servo to the subsystem using {@link HardwareMap}.
     *
     * @param hardwareMap    the hardware map to use
     * @param servoName      the servo's name
     * @param servoDirection the logical direction of the servo
     * @param offset         the offset to apply
     * @return this subsystem instance, for chaining
     */
    public ServoSubsystem withServo(HardwareMap hardwareMap, String servoName, Direction servoDirection, Double offset) {
        CuttleServo servo = new CuttleServo(hardwareMap, servoName).setOffset(offset).setDirection(servoDirection);
        servoList.add(servo);
        return this;
    }

    /**
     * Adds a servo to the subsystem using a {@link CuttleRevHub}.
     *
     * @param revHub         the rev hub to use
     * @param servoPort      the port number
     * @param servoDirection the logical direction
     * @param offset         the offset to apply
     * @return this subsystem instance, for chaining
     */
    public ServoSubsystem withServo(CuttleRevHub revHub, int servoPort, Direction servoDirection, Double offset) {
        CuttleServo servo = new CuttleServo(revHub, servoPort).setOffset(offset).setDirection(servoDirection);
        servoList.add(servo);
        return this;
    }
}
