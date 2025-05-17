package org.firstinspires.ftc.teamcode.Libraries.MMLib;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.Subsystem;


import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleRevHub;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.OpModeVeriables.AllianceColor;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.OpModeVeriables.AllianceSide;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.OpModeVeriables.OpModeType;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


/**
 * this class represents a wrapper for the default Teleop.
 * <p>
 * if it is used while calling the side and/or color val it initializes the MMRobot with the {@link OpModeType.Competition#TELEOP Teleop} type.
 * u might also use the constructor that only requires a {@link OpModeType.NonCompetition  NonComp} type,
 * this constructor let's u insert the type of {@link OpModeType.NonCompetition  NonComp} opmode u would like to use.
 * there are explanations in {@link OpModeType} that explains the 3 options u have.
 * there is the {@link OpModeType.NonCompetition#DEBUG Debug},
 * {@link OpModeType.NonCompetition#EXPERIMENTING Experimenting},
 * {@link OpModeType.NonCompetition#EXPERIMENTING_NO_EXPANSION Experimenting Without Expansion}.
 *
 */
public abstract class MMOpMode extends LinearOpMode {

    private final MMRobot mmRobot = MMRobot.getInstance();

    public OpModeType opModeType = null;

    public AllianceColor allianceColor;
    public AllianceSide allianceSide;

    private final List<Runnable> runOnInit = new ArrayList<>();
    private final List<Command> commandsOnRun = new ArrayList<>();

    /**
     * use this to choose a {@link OpModeType.NonCompetition NonComp} opmode.
     * @param opModeType which non-competition opmode to activate
     */
    public MMOpMode(OpModeType.NonCompetition opModeType) {
        this.opModeType = opModeType;
    }

    public MMOpMode() {
    }

    private void robotInit() {
        mmRobot.currentOpMode = this;
        MMRobot.getInstance().initializeSystems(opModeType.getOpModeType());
    }

    public abstract void onInit();

    public abstract void onInitLoop();

    public abstract void onPlay();

    public void onPlayLoop() {
        CommandScheduler.getInstance().run();                 //runs the scheduler
        MMSystems.getInstance().controlHub.pullBulkData();    //updates the controlHub sensors
        MMSystems.getInstance().expansionHub.pullBulkData();  //updates the expansionHub sensors
        telemetry.update();                                   //updates the telemetry
    }

    public abstract void onEnd();

    /**
     * Cancels all previous commands
     */
    public void reset() {
        CommandScheduler.getInstance().reset();
        MMRobot.getInstance().resetRobot();
    }

    public void addRunnableOnInit(Runnable... runOnInit) {
        this.runOnInit.addAll(Arrays.asList(runOnInit));
    }

    public void addCommandsOnRun(Command... commandsOnRun) {
        this.commandsOnRun.add(new InstantCommand().andThen(commandsOnRun));
        /*this was in order to solve the commandScheduler problem
          the problem was that the scheduler for some reason always ran the first instant command even tho it wasn't on yet*/
    }

    private void scheduleCommandsAndRun() {
        for(Runnable runnable : runOnInit) {
            runnable.run();
        }

        for(Command command : commandsOnRun) {
            schedule(command);
        }
    }

    /**
     * Schedules {@link com.seattlesolvers.solverslib.command.Command} objects to the scheduler
     */
    public void schedule(Command... commands) {
        CommandScheduler.getInstance().schedule(commands);
    }

    /**
     * Registers {@link com.seattlesolvers.solverslib.command.Subsystem} objects to the scheduler
     */
    public void register(Subsystem... subsystems) {
        CommandScheduler.getInstance().registerSubsystem(subsystems);
    }


    @Override
    public void runOpMode() throws InterruptedException {

        robotInit();
        onInit();
        scheduleCommandsAndRun();

        try {
            while (opModeInInit()) {
                onInitLoop();
            }
            onPlay();
            while (!isStopRequested() && opModeIsActive()) {
                onPlayLoop();
            }
        } finally {
            try {
                onEnd();
            } finally {
                reset();
            }
        }
    }


}
