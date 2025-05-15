package org.firstinspires.ftc.teamcode.Libraries.MMLib.Commands

import androidx.core.util.Supplier
import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.Subsystem
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.MMUtils

/**
 * ignore this (it was js for fun :p)
 */
private class MMDeferredCommandKT(
        commandSupplier: Supplier<Command>,
        vararg requiredSubsystems: Subsystem
) : CommandBase() {

    init {
        addRequirements(*requiredSubsystems)
    }

    private val nullCommand: Command = InstantCommand()
    private var supplier: Supplier<Command> =
            MMUtils.requireNonNullParam(commandSupplier, "Supplier<Command>", "MMDeferredCommand")
    private var command: Command = nullCommand

    override fun initialize() {
        val cmd = supplier.get()
        if(cmd != null) {
            command = cmd
        }
        command.initialize()
    }

    override fun execute() {
        command.execute()
    }

    override fun isFinished(): Boolean {
        return command.isFinished
    }

    override fun end(interrupted: Boolean) {
        command.end(interrupted)
        command = nullCommand
    }

}