package frc.robot.commands.elevator;

import frc.robot.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class JoystickElevator extends Command {

	private int positionIncrement = 300;
	private double signal = 0;
	public JoystickElevator() {
		requires(Robot.elevator);

	}

	// Called just before this Command runs the first time
	protected void initialize() {

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		// joystick control
		if(Robot.oi.getOperatorGamepad().getLeftYAxis()>0){
			signal = -0.3*Robot.oi.getOperatorGamepad().getLeftYAxis();
		}else{
			signal = -0.95*Robot.oi.getOperatorGamepad().getLeftYAxis();

		}
		// Robot.elevator.incrementTargetPosition((int) (signal * positionIncrement));
		// Robot.elevator.motionMagicControl();
		Robot.elevator.setElevator(ControlMode.PercentOutput, signal);

		// System.out.println("Elevator Velocity" +
		// Robot.elevator.elevatorLead.getSelectedSensorVelocity(0));
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return Math.abs(Robot.oi.getOperatorGamepad().getLeftYAxis())<0.1;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.elevator.setElevator(ControlMode.Position, Robot.elevator.getCurrentPosition(),DemandType.ArbitraryFeedForward,Robot.elevator.getArbitraryFeedForward());
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
