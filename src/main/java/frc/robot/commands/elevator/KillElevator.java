package frc.robot.commands.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 *
 */
public class KillElevator extends Command {

	public KillElevator() {
		requires(Robot.elevator);
	}

	
	protected void execute() {
		Robot.elevator.setElevator(ControlMode.PercentOutput, 0.0);
	}

	
	protected boolean isFinished() {
		return false;
	}

	protected void end() {
		System.out.println("~~~ELEVATOR EMERGENCY STOPPED~~~");
	}
}
