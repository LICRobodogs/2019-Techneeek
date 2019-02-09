package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmPistonState;

public class ArmPistonPosition extends Command {
	private ArmPistonState state;

	public ArmPistonPosition(ArmPistonState state) {
		requires(Robot.arm);
		this.state = state;
	}

	@Override
	protected void initialize() {
		Robot.arm.setArmPiston(state);
	}

	@Override
	protected void execute() {

	}

	@Override
	protected boolean isFinished() {
		return true;
	}

	@Override
	protected void end() {

	}

	@Override
	protected void interrupted() {

	}

}
