package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.OpModeVeriables.OpModeType;

@TeleOp
public class PortTest extends MMOpMode {
    Servo scoringElbow;
    Servo scoreArmR;
    Servo scoreArmL;

    public PortTest() {
        super(OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION);
    }

    @Override
    public void onInit() {
        scoringElbow = hardwareMap.servo.get("scoring elbow");
        scoreArmR = hardwareMap.servo.get("R outake arm");
        scoreArmL = hardwareMap.servo.get("L outake arm");
        scoreArmR.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void onInitLoop() {

    }

    @Override
    public void onPlay() {

    }

    @Override
    public void onPlayLoop() {
        scoreArmR.setPosition(gamepad1.left_trigger);
        scoreArmL.setPosition(gamepad1.left_trigger);
    }

    @Override
    public void onEnd() {

    }
}
