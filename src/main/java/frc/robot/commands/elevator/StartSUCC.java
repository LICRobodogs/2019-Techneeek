package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 *
 */
public class StartSUCC extends Command {
	
	protected void initialize() {
		Robot.elevator.startSUCC();
	}

	
	protected boolean isFinished() {
		return true;
	}

}
