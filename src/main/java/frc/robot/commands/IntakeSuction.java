package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Intake.IntakeState;

public class IntakeSuction extends Command {
	private IntakeState state;

	public IntakeSuction(IntakeState state) {
		requires(Robot.intake);
		this.state = state;
	}

	@Override
	protected void initialize() {
		System.out.println("suction " + state.toString());
		Robot.intake.setSuction(state);
		System.out.println("intake suction " + Robot.intake.getSuccState().toString());

	}

	@Override
	protected void execute() {

	}

	@Override
	protected boolean isFinished() {
		return false;
	}

	@Override
	protected void end() {

	}

	@Override
	protected void interrupted() {

	}

}
