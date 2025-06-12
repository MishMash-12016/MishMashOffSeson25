package org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Motor.Base;

import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleCrServo;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleMotor;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleRevHub;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.MMSubsystem;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.MMMotorOrCrServo;
import org.firstinspires.ftc.teamcode.MMRobot;
import java.util.ArrayList;

import Ori.Coval.Logging.WpiLog;

public class MotorOrCrServoSubsystem extends MMSubsystem {
    // List of motors or crServos driven by this subsystem
    private final ArrayList<MMMotorOrCrServo> motorOrCrServoList = new ArrayList<>();
    public final String subsystemName;
    public ZeroPowerBehavior zeroPowerBehavior;

    public MotorOrCrServoSubsystem(String subsystemName){
        this.subsystemName = subsystemName;
        MMRobot.getInstance().subsystems.add(this);
    }

    /**
     * Creates a Command that sets the motor or crServos power directly.
     *
     * @param power motor power (-1.0 to 1.0)
     * @return a RunCommand requiring this subsystem
     */
    public Command setPowerRunCommand(double power) {
        return new RunCommand(() -> setPower(power), this);
    }

    /**
     * Creates a Command that sets the motor or crServos power directly.
     *
     * @param power motor power (-1.0 to 1.0)
     * @return a RunCommand requiring this subsystem
     */
    public Command setPowerCommand(double power) {
        return new InstantCommand(() -> setPower(power), this);
    }

    /**
     * a command that stops the motors
     */
    public Command stopCommand(){
        return new InstantCommand(this::stop, this);
    }

    /**
     * @param power motor power (-1.0 to 1.0)
     * @apiNote !NOTICE THIS IS NOT A COMMAND AND WILL NOT STOP THE DEFAULT COMMAND
     * <p>Sets the raw power to all motors. Use with caution if a default command
     * is installed, as this method does not manage command requirements.</p>
     */
    public void setPower(double power) {
        WpiLog.log(subsystemName + "/power: ", power, true);

        for (MMMotorOrCrServo motor : motorOrCrServoList) {
            motor.setPower(power);
        }
    }

    public void stop(){
        setPower(0);
    }

    /**
     * adds a motor to this subsystem
     * @param revHub
     * @param port
     * @param direction
     */
    public MotorOrCrServoSubsystem withMotor(CuttleRevHub revHub, int port, Direction direction){
        MMMotorOrCrServo motor = new MMMotorOrCrServo(new CuttleMotor(revHub, port, direction));
        if(zeroPowerBehavior != null){
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }

        motorOrCrServoList.add(motor);
        return this;
    }

    public MotorOrCrServoSubsystem withCrServo(CuttleRevHub revHub, int port, Direction direction){
        MMMotorOrCrServo crServo = new MMMotorOrCrServo(new CuttleCrServo(revHub, port, direction));

        motorOrCrServoList.add(crServo);

        return this;
    }

    public MotorOrCrServoSubsystem withZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior){
        for (MMMotorOrCrServo motor: motorOrCrServoList){
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
        this.zeroPowerBehavior = zeroPowerBehavior;
        return this;
    }

    public MotorOrCrServoSubsystem withSetDefaultCommand(Command defaultCommand){
        setDefaultCommand(defaultCommand);
        return this;
    }

    @Override
    public void resetHub(){
        for(MMMotorOrCrServo motorOrCrServo : motorOrCrServoList){
            motorOrCrServo.resetHub();
        }
    }
}
