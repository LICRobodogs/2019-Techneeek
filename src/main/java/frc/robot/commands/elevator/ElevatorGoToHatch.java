package frc.robot.commands.elevator;

import frc.robot.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ElevatorGoToHatch extends Command {

	private int hatchLevel = 0;

	public ElevatorGoToHatch(int level) {
		requires(Robot.elevator);
		hatchLevel = Robot.elevator.getHatchPosition(level);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.elevator.setTargetPosition(hatchLevel);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.elevator.motionMagicControl();
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return Robot.elevator.isInPosition(hatchLevel);
	}

	// Called once after isFinished returns true
	protected void end() {
		// Robot.elevator.setElevator(ControlMode.Position,
		// Robot.elevator.getCurrentPosition(),DemandType.ArbitraryFeedForward,Robot.elevator.getArbitraryFeedForward());
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
