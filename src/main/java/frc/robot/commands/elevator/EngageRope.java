package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 *
 */
public class EngageRope extends Command {
	
	protected void initialize() {
		Robot.elevator.engageRope();
	}

	
	protected boolean isFinished() {
		return true;
	}

}
