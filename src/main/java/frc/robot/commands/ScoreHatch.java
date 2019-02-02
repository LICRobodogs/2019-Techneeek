package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.subsystems.Arm.ArmPistonState;
import frc.robot.subsystems.Intake.IntakeState;

public class ScoreHatch extends CommandGroup {
	public ScoreHatch() {
		addSequential(new IntakeSuction(IntakeState.SUCC_OUT));
        addSequential(new WaitCommand(0.45));
        addSequential(new ArmPistonPosition(ArmPistonState.SHOOT));
        addSequential(new WaitCommand(1));
        addSequential(new ArmPistonPosition(ArmPistonState.RELOAD));
	}
}
