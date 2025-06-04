package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMRobotInternals;

public class MMRobot extends MMRobotInternals{
    private static MMRobot instance;

    public static synchronized MMRobot getInstance() {
        if (instance == null) {
            instance = new MMRobot();
        }
        return instance;
    }

    public synchronized void resetRobot(){
        instance = null;
        MMSystems.getInstance().resetSystems();
    }

    @Override
    public void initAuto() {
        MMSystems.getInstance().initBasics();
    }

    @Override
    public void initTele() {
        MMSystems.getInstance().initBasics();
    }

    @Override
    public void initDebug() {
    }
}
