package org.firstinspires.ftc.teamcode.Libraries.MMLib.Commands;

import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.SubsystemStructure.IMMPositionSubsystem;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.SubsystemStructure.IMMPowerSubsystem;

/**
 * this command is similar to the {@link MMToggleCommand},
 * while using a more specific input of the {@link com.arcrobotics.ftclib.command.SubsystemBase Subsystem}
 * @param <T> the type of ur subsystem (probably Double)
 */
public class MMToggleCommand2<T> extends StartEndCommand {
    public MMToggleCommand2(IMMPowerSubsystem<T> subsystem, T on, T off, Subsystem... requirements) {
        super(
                () -> subsystem.setPower(on),
                () -> subsystem.setPower(off),
                requirements
        );
    }

    public MMToggleCommand2(IMMPositionSubsystem<T> subsystem, T on, T off, Subsystem... requirements) {
        super(
                () -> subsystem.setPosition(on),
                () -> subsystem.setPosition(off),
                requirements
        );
    }

}
