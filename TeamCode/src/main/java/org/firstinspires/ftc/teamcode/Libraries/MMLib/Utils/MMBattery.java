package org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * this class represents ur robot battery
 */
public class MMBattery {

    private final VoltageSensor battery;

    public MMBattery(HardwareMap hardwareMap) {
        this.battery = hardwareMap.voltageSensor.iterator().next();
    }

    /**
     * @return the battery voltage.
     */
    public double getVoltage() {
        return battery.getVoltage();
    }

    /**
     * the ftc 12V batteries range between 11V as the lowest, and 15V (usually 14.5V is more common) are the highest.
     * @return how suitable the battery is for competition.
     * <p>
     * (100% - very suitable, 0% - DO NOT USE)
     */
    public double getPercentage() {
        return getPercentage(11, 14.5);
    }

    /**
     * this method can be used if u want to change the highest and lowest.
     * @param lowest can NOT go to comp with this
     * @param highest great for comp
     * @return how suitable the battery is for competition.
     * <p>
     * (100% - very suitable, 0% - DO NOT USE)
     */
    public double getPercentage(double lowest, double highest) {
        return MMUtils.mapValuesLinearByRange(
                getVoltage(),
                new MMRange(lowest, highest),
                new MMRange(0, 100) //turn into numbers that range from 0 to 100 (percentage).
        );
    }

}
