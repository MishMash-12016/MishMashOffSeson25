package org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Motor.Base;

import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleMotor;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleRevHub;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;

import java.util.ArrayList;

public class MotorSubsystem extends SubsystemBase {
    // List of motors driven by this subsystem
    private final ArrayList<CuttleMotor> motorList = new ArrayList<>();
    public final String subsystemName;
    public ZeroPowerBehavior zeroPowerBehavior;

    public MotorSubsystem(String subsystemName){
        this.subsystemName = subsystemName;
    }

    /**
     * Creates a Command that sets the motor power directly.
     *
     * @param power motor power (-1.0 to 1.0)
     * @return a RunCommand requiring this subsystem
     */
    public Command setPowerRunCommand(double power) {
        return new RunCommand(() -> setPower(power), this);
    }

    /**
     * Creates a Command that sets the motor power directly.
     *
     * @param power motor power (-1.0 to 1.0)
     * @return a RunCommand requiring this subsystem
     */
    public Command setPowerCommand(double power) {
        return new InstantCommand(() -> setPower(power), this);
    }

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
        for (CuttleMotor motor : motorList) {
            motor.setPower(power);
        }
    }

    public void stop(){
        setPower(0);
    }

    public MotorSubsystem withMotor(CuttleRevHub revHub, int port, Direction direction){
        CuttleMotor motor = new CuttleMotor(revHub, port, direction);
        if(zeroPowerBehavior != null){
            motor.setZeroPowerBehaviour(zeroPowerBehavior);
        }

        motorList.add(motor);
        return this;
    }

    public MotorSubsystem withZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior){
        for (CuttleMotor motor:motorList){
            motor.setZeroPowerBehaviour(zeroPowerBehavior);
        }
        this.zeroPowerBehavior = zeroPowerBehavior;
        return this;
    }

    public MotorSubsystem withSetDefaultCommand(Command defaultCommand){
        setDefaultCommand(defaultCommand);
        return this;
    }
}
