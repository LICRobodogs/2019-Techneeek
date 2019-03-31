package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 *
 */
public class EngageGravity extends Command {

	
	protected void initialize() {
		Robot.elevator.engageGravity();
		SmartDashboard.putBoolean("DONE SUCC", false);
	}

	
	protected boolean isFinished() {
		return true;
	}

}
