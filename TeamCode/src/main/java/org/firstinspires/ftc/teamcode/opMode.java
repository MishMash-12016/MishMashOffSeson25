package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.OpModeVeriables.OpModeType;
import org.firstinspires.ftc.teamcode.Subsystems.TestSubsystem;

import Ori.Coval.Logging.WpiLog;

@TeleOp
public class opMode extends MMOpMode {

    ElapsedTime elapsedTime;

    public opMode() {
        super(OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION);
    }

    @Override
    public void onInit() {
//        TestSubsystem.instance = null;
        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
    }

    @Override
    public void onInitLoop() {
        WpiLog.log("init loop time" ,elapsedTime.milliseconds(), false);
        elapsedTime.reset();
    }

    @Override
    public void onPlay() {
        TestSubsystem.getInstance().setPosition(0.5);
    }

    @Override
    public void onPlayLoop() {
        TestSubsystem.getInstance().move();
        WpiLog.log("play loop time" ,elapsedTime.milliseconds(), false);
        elapsedTime.reset();

    }

    @Override
    public void onEnd() {

    }
}
