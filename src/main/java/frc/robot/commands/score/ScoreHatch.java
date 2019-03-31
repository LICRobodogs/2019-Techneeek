package frc.robot.commands.score;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.commands.arm.ArmPistonPosition;
import frc.robot.commands.intake.IntakeSuction;
import frc.robot.subsystems.Arm.ArmPistonState;
import frc.robot.subsystems.Intake.IntakeState;

public class ScoreHatch extends CommandGroup {
    public ScoreHatch() {
        addSequential(new IntakeSuction(IntakeState.SUCC_OUT));
        addSequential(new WaitCommand(0.2));
        addSequential(new ArmPistonPosition(ArmPistonState.SHOOT));
        addSequential(new WaitCommand(1.5));
        addSequential(new ArmPistonPosition(ArmPistonState.RELOAD));
    }
}
