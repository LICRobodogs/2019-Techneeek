package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Turns to the largest contour in view
 */
public class TurnToTarget extends Command {
	public TurnToTarget() {
		requires(Robot.driveTrain);
	}

	protected void execute() {
		Robot.limeLight.aimAtTarget();
	}

	protected boolean isFinished() {
		return Robot.limeLight.isAimedAtTarget();
	}

}
