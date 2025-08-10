package org.firstinspires.ftc.teamcode.CommendGroups;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Subsystems.ScoringArm;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringClaw;

public class Intake {
    public static Command elevateArm(){
        return new SequentialCommandGroup(
                ScoringArm.getInstance().setPositionCommand(ScoringArm.scoringArmPark),
                ScoringClaw.getInstance().setPositionCommand(ScoringClaw.IntakeClawOpenPos)
        );
    }
}
