package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleRevHub;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.MMBattery;
import org.firstinspires.ftc.teamcode.Libraries.pedroPathing.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.Libraries.pedroPathing.pedroPathing.constants.LConstants;

//TODO:everything(to build a class you first need to invent the entire universe)
public class MMSystems {

    public void initTeleopSystems(){

    }

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

    public CuttleRevHub controlHub;
    public CuttleRevHub expansionHub;
    public MMBattery battery;

    //pedroPathing
    public Timer pathTimer;
    public Follower follower;




    public Follower initializeFollower(Pose pose) {
        pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(MMRobot.getInstance().currentOpMode.hardwareMap);
        follower.setStartingPose(pose);
        return follower;
    }

}
