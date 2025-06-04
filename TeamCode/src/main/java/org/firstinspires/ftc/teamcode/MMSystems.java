package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleRevHub;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.MMBattery;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.OpModeVeriables.OpModeType;

//TODO:everything(to build a class you first need to invent the entire universe)
public class MMSystems {

    //basic robot things(control hub, expansion hub...)
    public CuttleRevHub controlHub;
    public CuttleRevHub expansionHub;
    public MMBattery battery;
    public GamepadEx gamepadEx1;
    public GamepadEx gamepadEx2;

    private static MMSystems instance;

    public static synchronized MMSystems getInstance() {
        if (instance == null) {
            instance = new MMSystems();
        }
        return instance;
    }

    public synchronized void resetSystems(){
        instance = null;
    }

    private MMSystems() {
        initBasics();
    }


    //pre made init for pedro and robot basics because they shouldn't change and be the same every year
    //!!!!do not create here init for subsystems or other robot things do that in MMRobot

    public void initBasics() {
        HardwareMap hardwareMap = MMRobot.getInstance().currentOpMode.hardwareMap;
        gamepadEx1 = new GamepadEx(MMRobot.getInstance().currentOpMode.gamepad1);
        gamepadEx2 = new GamepadEx(MMRobot.getInstance().currentOpMode.gamepad2);

        if(controlHub == null || MMRobot.getInstance().currentOpMode.opModeType != OpModeType.Competition.TELEOP) {
            controlHub = new CuttleRevHub(hardwareMap, CuttleRevHub.HubTypes.CONTROL_HUB);
            if (MMRobot.getInstance().currentOpMode.opModeType != OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION) {
                expansionHub = new CuttleRevHub(hardwareMap, CuttleRevHub.HubTypes.EXPANSION_HUB);
            }
            battery = new MMBattery(hardwareMap);
        }
    }
}
