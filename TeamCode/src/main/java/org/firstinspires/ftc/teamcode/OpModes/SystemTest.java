package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.OpModeVeriables.OpModeType;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringArm;

@TeleOp
public class SystemTest extends MMOpMode {

    public SystemTest() {
        super(OpModeType.NonCompetition.DEBUG);
    }

    @Override
    public void onInit() {
       MMRobot.getInstance().gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                ScoringArm.getInstance().setPositionCommand(ScoringArm.scoringArmScoreSample)
        );

        MMRobot.getInstance().gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                ScoringArm.getInstance().setPositionCommand(ScoringArm.scoringArmInitPose)
        );
    }

    @Override
    public void onInitLoop() {

    }

    @Override
    public void onPlay() {
    }

    @Override
    public void onPlayLoop() {

    }

    @Override
    public void onEnd() {

    }
}