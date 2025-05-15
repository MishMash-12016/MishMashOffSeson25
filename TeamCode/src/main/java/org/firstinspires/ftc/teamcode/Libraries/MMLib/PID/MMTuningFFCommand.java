package org.firstinspires.ftc.teamcode.Libraries.MMLib.PID;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MMRobot;

/**
 * this command can help u find the FF value in ur system.
 * <p>
 * this command requires a {@link MMPIDSubsystem} and a threshold to reach.
 * on the execute stage it keeps on adding power to the subsystem in small jumps (which u can change urself),
 * and once the subsystem's {@link MMPIDSubsystem#getCurrentValue()} returns a value that the distance from the origin point, which is
 * default to be 0 (but can be changed), is more than the threshold, it uses the {@link MMPIDSubsystem#stop()} method
 * (which u should make sure u implemented),
 * and sends to ur telemetry the minPower to pass this threshold. this power should be ur kF value.
 * </p>
 * make sure u have an {@link Telemetry#update()} method in ur opmode's loop
 */
public class MMTuningFFCommand extends CommandBase {

    MMRobot mmRobot = MMRobot.getInstance();

    private final MMPIDSubsystem subsystem;

    private double power = 0;
    private double jumps = 0.0002;
    private final double threshold;
    private double startingPoint = 0;

    /**
     * use this if the default power jumps (0.0002) and starting point (0) is ok with ur system
     * @param subsystem the {@link MMPIDSubsystem} u want to tune
     * @param threshold this threshold is used to know that ur system has moved
     */
    public MMTuningFFCommand(MMPIDSubsystem subsystem, double threshold) {
        this.subsystem = subsystem;
        this.threshold = threshold;
        addRequirements(subsystem);
    }

    /**
     * use this in case u want to change the power jumps
     * @param subsystem the {@link MMPIDSubsystem} u want to tune
     * @param threshold this threshold is used to know that ur system has moved
     * @param powerJumps the power jumps in each iteration
     */
    public MMTuningFFCommand(MMPIDSubsystem subsystem, double threshold, double powerJumps) {
        this(subsystem, threshold);
        jumps = powerJumps;
    }

    /**
     * use this if u want to change the default power jumps AND the starting point
     * @param subsystem the {@link MMPIDSubsystem} u want to tune
     * @param threshold this threshold is used to know that ur system has moved
     * @param powerJumps the power jumps in each iteration
     * @param startingPoint the resting point of ur system
     */
    public MMTuningFFCommand(MMPIDSubsystem subsystem, double threshold, double powerJumps, double startingPoint) {
        this(subsystem, threshold, powerJumps);
        this.startingPoint = startingPoint;
    }

    /**
     * calculate the distance between the {@link MMPIDSubsystem#getCurrentValue()} and the {@link #startingPoint}
     * @return the distance between the current value and the starting point
     */
    private double error() {
        return Math.abs(subsystem.getCurrentValue() - startingPoint);
    }

    /**
     * returns whether the threshold has passed, true if its not less than the threshold anymore
     * @return whether the threshold has reached
     */
    private boolean isThresholdReached() {
        return !(error() < threshold);
    }

    /**
     * updates the power of the subsystem in case the threshold hasn't been reached yet.
     * <p>
     * make sure u have the {@link Telemetry#update()} method in the opmode's loop,
     * otherwise u wont see it.
     */
    @Override
    public void execute() {
        if(!isThresholdReached()) {
            power += jumps;
        }
        subsystem.setPower(power);

        /*add telemetry*/
        mmRobot.mmSystems.telemetry.addLine("TuningFFCommand is in progress...");
        mmRobot.mmSystems.telemetry.addData("progress left", error() + " / " + threshold);
        mmRobot.mmSystems.telemetry.addData("progress left (%)", (error()/threshold)*100 + "%");
        mmRobot.mmSystems.telemetry.addData("power", power);

    }

    @Override
    public boolean isFinished() {
        return isThresholdReached();
    }

    /**
     * this method calls the {@link MMPIDSubsystem#stop()} method,
     * make sure u implemented it, otherwise ur subsystem might NOT stop!
     * <p>
     * this also sends the minPower to ur telemetry.
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        subsystem.stop(); //this wouldn't work if the user didn't override it
        power = 0;

        /*add telemetry of minPower*/
        mmRobot.mmSystems.telemetry.setAutoClear(false); //autoClear is set to be false so u wont lose the minPower.
        mmRobot.mmSystems.telemetry.addLine("TuningFFCommand is done. (make sure ur system is stopped)");
        mmRobot.mmSystems.telemetry.addData("minPower", power);

    }
}
