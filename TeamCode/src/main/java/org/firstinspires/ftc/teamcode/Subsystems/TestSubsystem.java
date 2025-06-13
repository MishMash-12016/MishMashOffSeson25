package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Servo.ServoSubsystem;
import org.firstinspires.ftc.teamcode.MMRobot;

import Ori.Coval.Logging.AutoLog;
import Ori.Coval.Logging.AutoLogAndPostToFtcDashboard;

@AutoLogAndPostToFtcDashboard
public class TestSubsystem extends ServoSubsystem {
    public int ndsfjhf = 3;

    public static TestSubsystem instance;

    public static synchronized TestSubsystem getInstance() {
        if (instance == null) {
            instance = new TestSubsystemAutoLogged();
        }
        return instance;
    }

    public TestSubsystem() {
        super("testServo");
        withServo(5, MMRobot.getInstance().controlHub, Direction.FORWARD, 0);
    }

    public double move(){
        return 3;
    }
}
