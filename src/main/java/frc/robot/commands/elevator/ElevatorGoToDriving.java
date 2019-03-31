package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 *
 */
public class ElevatorGoToDriving extends Command {

	public ElevatorGoToDriving() {
		requires(Robot.elevator);
	}

	
	protected void initialize() {
		super.setTimeout(5);
		Robot.elevator.setTargetPosition(Robot.elevator.getDrivingPosition());
	}

	
	protected void execute() {
		Robot.elevator.motionMagicControl();
	}

	
	protected boolean isFinished() {
		return Robot.elevator.isInPosition(Robot.elevator.getDrivingPosition());
	}
}
