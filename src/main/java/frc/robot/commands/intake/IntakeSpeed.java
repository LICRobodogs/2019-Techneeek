package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class IntakeSpeed extends Command {

	public double mSpeed, mSpeed2;

	public IntakeSpeed() {
		requires(Robot.intake);
	}

	public IntakeSpeed(double speed) {
		requires(Robot.intake);
		this.mSpeed = speed;
		this.mSpeed2 = 0;
	}

	public IntakeSpeed(double speed1, double speed2) {
		requires(Robot.intake);
		this.mSpeed = speed1;
		this.mSpeed2 = speed2;
	}

	protected void execute() {
		Robot.intake.setSpeed(mSpeed);
	}

	@Override
	protected boolean isFinished() {
		return true;
	}

}
