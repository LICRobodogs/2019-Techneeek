package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Command to drive to a hatch
 */
public class DriveToTarget extends Command {

	public DriveToTarget() {
		requires(Robot.driveTrain);
	}

	protected void initialize() {
		setTimeout(5.0);
		System.out.println("starting command");
	}

	protected void execute() {
		if (Robot.limeLight.is3dCompute()) {
			Robot.limeLight.driveStraightToTarget();
		}
	}

	protected boolean isFinished() {
		return Robot.limeLight.isAtTarget();
	}

	protected void end() {
		System.out.println("done driving command");
		Robot.driveTrain.setSpeed(0);
	}

	protected void interrupted() {
		System.out.println("interrupted driving command");
		Robot.driveTrain.setSpeed(0);
	}
}
