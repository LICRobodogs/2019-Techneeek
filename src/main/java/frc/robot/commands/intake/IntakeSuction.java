package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Intake.IntakeState;

public class IntakeSuction extends Command {
	private IntakeState state;

	public IntakeSuction(IntakeState state) {
		this.state = state;
	}

	@Override
	protected void initialize() {
		Robot.intake.setSuction(state);
	}

	@Override
	protected boolean isFinished() {
		return true;
	}
}
