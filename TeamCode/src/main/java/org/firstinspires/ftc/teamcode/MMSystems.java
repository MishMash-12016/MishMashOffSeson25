package org.firstinspires.ftc.teamcode;

//TODO:everything(to build a class you first need to invent the entire universe)
public class MMSystems {
    /**
     * the robot instance
     */
    private static MMSystems instance;

    /**
     * the get method for the singleton
     * @return the robot instance
     */
    public static synchronized MMSystems getInstance() {
        if (instance == null) {
            instance = new MMSystems();
        }
        return instance;
    }


}
