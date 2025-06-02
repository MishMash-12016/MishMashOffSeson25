package org.firstinspires.ftc.teamcode.Libraries.MMLib;

import com.seattlesolvers.solverslib.command.Robot;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.OpModeVeriables.OpModeType;

public abstract class MMRobotInternals extends Robot {

    public MMOpMode currentOpMode;

    /**
     * this initializes your subsystems.
     * <p>
     * if experimenting, then this does nothing.
     * @param type the {@link OpModeType} chosen
     */
    public void initializeSystems(OpModeType type) {
        if(type == OpModeType.Competition.TELEOP) {
            initTele();
        } else if (type == OpModeType.Competition.AUTO) {
            initAuto();
        } else if(type == OpModeType.NonCompetition.DEBUG) {
            initDebug();
        }
    }


    /**
     * initialize subsystems required for auto
     */
    public abstract void initAuto();

    /**
     * initialize subsystems required for teleop
     */
     public abstract void initTele();

    /**
     * initialize subsystems required for debug mode
     */
    public abstract void initDebug();
}
