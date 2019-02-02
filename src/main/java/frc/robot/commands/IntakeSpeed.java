package frc.robot.commands;

import frc.robot.*;
import edu.wpi.first.wpilibj.command.Command;

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
		this.mSpeed = speed1;
		this.mSpeed2 = speed2;
	}

	@Override
	protected void initialize() {

	}

	protected void execute() {
		if (mSpeed != 0 && mSpeed2 != 0) {
			Robot.intake.setTopSpeed(mSpeed);
			Robot.intake.setBottomSpeed(mSpeed2);
		} else if (mSpeed != 0) {
			Robot.intake.setSpeed(mSpeed);
		} else {
			Robot.intake.setSpeed();
		}
		// SmartDashboard.putNumber("Intake Speed: ",mSpeed);
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return true;
	}

	protected void end() {
		// mSpeed = 0;
	}

}
