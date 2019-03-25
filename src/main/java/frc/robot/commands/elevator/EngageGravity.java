package frc.robot.commands.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;

/**
 *
 */
public class EngageGravity extends Command {

	public EngageGravity() {
	}

	// Called just before this Command runs the first time
	protected void initialize() {
        Robot.elevator.engageGravity();
        SmartDashboard.putBoolean("DONE SUCC",false);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
        return true;
	}

	// Called once after isFinished returns true
	protected void end() {
    }


	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
