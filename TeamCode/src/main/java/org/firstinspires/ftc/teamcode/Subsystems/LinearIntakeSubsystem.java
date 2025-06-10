package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.Command;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Servo.ServoSubsystem;
import org.firstinspires.ftc.teamcode.MMRobot;

import Ori.Coval.Logging.AutoLogAndPostToFtcDashboard;

@Config
@AutoLogAndPostToFtcDashboard
public class LinearIntakeSubsystem extends ServoSubsystem {

    public static int SERVO_PORT_1 = 0;
    public static int SERVO_PORT_2 = 1;
    public static Direction SERVO_DIRECTION_1 = Direction.FORWARD;
    public static Direction SERVO_DIRECTION_2 = Direction.REVERSE;
    public static double SERVO_OFFSET_1 = 0.0;
    public static double SERVO_OFFSET_2 = 0.015;
    public static double OPEN_POSITION = 0.75;
    public static double CLOSE_POSITION = 0.0;


    // Singleton instance
    public static LinearIntakeSubsystemAutoLogged instance;
    /**
     * Get the singleton instance of LinearIntakeSubsystem.
     */
    public static synchronized LinearIntakeSubsystemAutoLogged getInstance() {
        if (instance == null) {
            instance = new LinearIntakeSubsystemAutoLogged("LinearIntakeSubsystem");
        }
        return instance;
    }

    public LinearIntakeSubsystem(String subsystemName) {
        super(subsystemName);

        MMRobot mmRobot = MMRobot.getInstance();
        withServo(SERVO_PORT_1, mmRobot.controlHub, SERVO_DIRECTION_1, SERVO_OFFSET_1);
        withServo(SERVO_PORT_2, mmRobot.controlHub, SERVO_DIRECTION_2, SERVO_OFFSET_2);
    }

    public Command openCommand() {
        return setPositionCommand(OPEN_POSITION);
    }

    public Command closeCommand() {
        return setPositionCommand(CLOSE_POSITION);
    }
}

