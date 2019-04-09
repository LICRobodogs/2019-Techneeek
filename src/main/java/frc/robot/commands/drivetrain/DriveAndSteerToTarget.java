package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Command to drive to a hatch
 */
public class DriveAndSteerToTarget extends Command {
	int lostCount;

	public DriveAndSteerToTarget() {
		requires(Robot.driveTrain);
	}

	protected void initialize() {
		lostCount = 0;
		setTimeout(5.0);
		System.out.println("starting command");
	}

	protected void execute() {
		if (Robot.limeLight.is3dCompute()) {
			lostCount = 0;
			Robot.limeLight.drive_and_steer();
		} else {
			lostCount++;
		}
	}

	protected boolean isFinished() {
		return lostCount >= 5 || (Robot.limeLight.isAtTarget() && Robot.limeLight.isAimedAtTarget());
		// return lostCount >= 5 || (Robot.limeLight.isAtTarget());
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
