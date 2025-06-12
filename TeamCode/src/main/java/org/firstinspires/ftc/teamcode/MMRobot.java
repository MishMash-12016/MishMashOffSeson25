package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleRevHub;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.MMSubsystem;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.MMBattery;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.OpModeVeriables.OpModeType;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMDrivetrain;

import java.util.ArrayList;
import java.util.List;

public class MMRobot extends Robot {
    public MMOpMode currentOpMode;

    //basic robot things(control hub, expansion hub...)
    public CuttleRevHub controlHub;
    public CuttleRevHub expansionHub;
    public MMBattery battery;
    public GamepadEx gamepadEx1;
    public GamepadEx gamepadEx2;

    public ArrayList<MMSubsystem> subsystems = new ArrayList<>();

    private static MMRobot instance;

    public static synchronized MMRobot getInstance() {
        if (instance == null) {
            instance = new MMRobot();
        }
        return instance;
    }

    public MMRobot() {
    }

    /**
     * this initializes your subsystems.
     * <p>
     * if experimenting, then this does nothing.
     *
     * @param type the {@link OpModeType} chosen
     */
    public void initializeSystems(OpModeType type) {
        initBasics();
        MMDrivetrain.init();

        if (type == OpModeType.Competition.TELEOP) {
            initTele();
        } else if (type == OpModeType.Competition.AUTO) {
            initAuto();
        } else if (type == OpModeType.NonCompetition.DEBUG) {
            initDebug();
        }
    }


    public synchronized void resetRobot() {
        instance = null;
    }

    public void initAuto() {
    }

    public void initTele() {
    }

    public void initDebug() {
    }

    public void initSubsystems(){
        for (MMSubsystem subsystem : subsystems){
            subsystem.resetHub();
        }
    }

    private void initBasics() {
        HardwareMap hardwareMap = MMRobot.getInstance().currentOpMode.hardwareMap;
        gamepadEx1 = new GamepadEx(MMRobot.getInstance().currentOpMode.gamepad1);
        gamepadEx2 = new GamepadEx(MMRobot.getInstance().currentOpMode.gamepad2);

        controlHub = new CuttleRevHub(hardwareMap, CuttleRevHub.HubTypes.CONTROL_HUB);
        if (MMRobot.getInstance().currentOpMode.opModeType != OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION) {
            expansionHub = new CuttleRevHub(hardwareMap, CuttleRevHub.HubTypes.EXPANSION_HUB);
        }
        battery = new MMBattery(hardwareMap);
    }
}
