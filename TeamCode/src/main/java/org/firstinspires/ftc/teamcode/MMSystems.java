package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleRevHub;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.MMBattery;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.OpModeVeriables.OpModeType;
import org.firstinspires.ftc.teamcode.Libraries.pedroPathing.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.Libraries.pedroPathing.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeArmSubsystem;

//TODO:everything(to build a class you first need to invent the entire universe)
public class MMSystems {

    //basic robot things(control hub, expansion hub...)
    public CuttleRevHub controlHub;
    public CuttleRevHub expansionHub;
    public MMBattery battery;
    public GamepadEx gamepadEx1;
    public GamepadEx gamepadEx2;

    //pedroPathing
    public Timer pathTimer;
    public Follower follower;


    private static MMSystems instance;

    public static synchronized MMSystems getInstance() {
        if (instance == null) {
            instance = new MMSystems();
        }
        return instance;
    }


    //pre made init for pedro and robot basics because they shouldn't change and be the same every year
    //!!!!do not create here init for subsystems or other robot things do that in MMRobot

    public void initBasics() {
        HardwareMap hardwareMap = MMRobot.getInstance().currentOpMode.hardwareMap;
        controlHub = new CuttleRevHub(hardwareMap, CuttleRevHub.HubTypes.CONTROL_HUB);
        if (MMRobot.getInstance().currentOpMode.opModeType.getOpModeType() != OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION) {
            expansionHub = new CuttleRevHub(hardwareMap, CuttleRevHub.HubTypes.EXPANSION_HUB);
        }
        battery = new MMBattery(hardwareMap);

        gamepadEx1 = new GamepadEx(MMRobot.getInstance().currentOpMode.gamepad1);
        gamepadEx2 = new GamepadEx(MMRobot.getInstance().currentOpMode.gamepad2);
    }

    public Follower initializeFollower(Pose pose) {
        pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(MMRobot.getInstance().currentOpMode.hardwareMap);
        follower.setStartingPose(pose);
        return follower;
    }

}
