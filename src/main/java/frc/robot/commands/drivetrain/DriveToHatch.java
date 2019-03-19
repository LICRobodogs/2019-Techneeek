package frc.robot.commands.drivetrain;

import frc.robot.*;
import frc.robot.subsystems.LimeLight.TargetType;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Command to drive to a hatch
 */
public class DriveToHatch extends Command {

	public DriveToHatch() {
		requires(Robot.driveTrain);
		requires(Robot.limeLight);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		setTimeout(5.0);
		System.out.println("starting command");
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.limeLight.getInHatchRange();
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return Robot.limeLight.isAtTarget(TargetType.HATCH);
	}

	// Called once after isFinished returns true
	protected void end() {
		System.out.println("done driving command");
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		System.out.println("interrupted driving command");
	}
}
