package org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Cuttlefish compatible servo.
 * Contains a list of preset positions which can be appended to using the addPreset() function.
 *
 * */
public class CuttleServo{

    private double pos = 0.0;
    public int port;
    boolean enabled = false;
    final boolean FTCServo;

    boolean isReverse = false;
    CuttleRevHub hub;
    com.qualcomm.robotcore.hardware.Servo ftcServoDevice;

    /**
     * Initialize servo using cuttlefish direct access system
     * @param revHub
     * @param servoPort
     * */
    public CuttleServo(CuttleRevHub revHub, int servoPort)
    {
        port = servoPort;
        hub = revHub;
        FTCServo = false;
    }
    /**
     * Initialize servo using hardwareMap
     * @param hardwareMap hardwareMap object
     * @param name Name of the servo in the config
     * */
    public CuttleServo(HardwareMap hardwareMap, String name)
    {
        FTCServo = true;
        ftcServoDevice = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class,name);

    }

    /**
     * Set the target position of the servo
     * @param position Target position
     * */
    public void setPosition(double position) {
        pos = isReverse? 1 - position : position;
        if(!FTCServo)
        {
            hub.setServoPosition(port,pos);
            if(!enabled)
            {
                enablePWM(true);
            }
        }
        else
        {
            ftcServoDevice.setPosition(position);
        }
    }

    /**
     * Enable or disable PWM on the servo port. This will not work if the servo was obtained using hardwareMap.
     * @param  enable If set to true PWM will be enabled, and if set to false PWM will be disabled
     * */
    public void enablePWM(boolean enable)
    {
        if(!FTCServo)
        {
            hub.enableServoPWM(port,enable);
            enabled = enable;
        }
    }

    /**
     * Get the target position of the servo.
     * <br>
     * IMPORTANT: This will not give the actual position of the servo.
     * */
    public double getPosition() {
        return pos;
    }

    public void setReverse(boolean isReverse) {
        this.isReverse = isReverse;
    }
}
