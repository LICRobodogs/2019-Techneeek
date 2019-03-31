package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 *
 */
public class KillDrive extends Command {

	public KillDrive() {
		requires(Robot.driveTrain);
	}

	protected void execute() {
		Robot.driveTrain.setSpeed(0.0);
	}

	protected boolean isFinished() {
		return false;
	}

	protected void end() {
		System.out.println("~~~DRIVE EMERGENCY STOPPED~~~");
	}

}
