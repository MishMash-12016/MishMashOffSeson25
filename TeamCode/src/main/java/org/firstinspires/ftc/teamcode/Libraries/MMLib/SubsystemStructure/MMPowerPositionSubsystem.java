package org.firstinspires.ftc.teamcode.Libraries.MMLib.SubsystemStructure;

import com.arcrobotics.ftclib.command.SubsystemBase;

public abstract class MMPowerPositionSubsystem<POWER, POSITION>
        extends SubsystemBase
        implements IMMPowerSubsystem<POWER>, IMMPositionSubsystem<POSITION> {
}
