package org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Servo;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleCrServo;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleRevHub;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;

import java.util.ArrayList;

public class CrServoSubsystem extends SubsystemBase {
    ArrayList<CuttleCrServo> servoList = new ArrayList<>();

    public CrServoSubsystem(CuttleRevHub revHub, int servoPort, Direction servoDirection) {
        CuttleCrServo servo = new CuttleCrServo(revHub, servoPort).setDirection(servoDirection);
        servoList.add(servo);
    }

    public CrServoSubsystem(HardwareMap hardwareMap, String servoName, Direction servoDirection) {
        CuttleCrServo servo = new CuttleCrServo(hardwareMap, servoName).setDirection(servoDirection);
        servoList.add(servo);
    }

    public CrServoSubsystem(CuttleCrServo servo){
        servoList.add(servo);
    }

    public Command setPowerCommand(double power){
        return new InstantCommand(()-> setPower(power), this);
    }

    public void setPower(double power){
        for (CuttleCrServo servo:servoList){
            servo.setPower(power);
        }
    }

    public double getPower(){
        return servoList.get(0).getPower();
    }

    public CrServoSubsystem withServo(CuttleRevHub revHub, int servoPort, Direction servoDirection) {
        CuttleCrServo servo = new CuttleCrServo(revHub, servoPort).setDirection(servoDirection);
        servoList.add(servo);
        return this;
    }

    public CrServoSubsystem withServo(HardwareMap hardwareMap, String servoName, Direction servoDirection) {
        CuttleCrServo servo = new CuttleCrServo(hardwareMap, servoName).setDirection(servoDirection);
        servoList.add(servo);
        return this;
    }

    public CrServoSubsystem withServo(CuttleCrServo servo){
        servoList.add(servo);
        return this;
    }
}
