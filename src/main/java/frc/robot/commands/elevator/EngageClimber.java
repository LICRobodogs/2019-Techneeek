package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 *
 */
public class EngageClimber extends Command {
	
	protected void initialize() {
		Robot.elevator.engageClimber();
	}

	
	protected boolean isFinished() {
		return true;
	}
}
