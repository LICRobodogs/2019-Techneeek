package frc.robot.commands.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 *
 */
public class JoystickElevator extends Command {

	private double signal = 0;

	public JoystickElevator() {
		requires(Robot.elevator);
	}



	
	protected void execute() {
//TODO ask nikita about this commented logic

		// joystick control
		if (Robot.oi.getOperatorGamepad().getLeftYAxis() > 0) {
			signal = -0.3 * Robot.oi.getOperatorGamepad().getLeftYAxis();
		} else {
			signal = -0.95 * Robot.oi.getOperatorGamepad().getLeftYAxis();
		}
		// Robot.elevator.incrementTargetPosition((int) (signal * positionIncrement));
		// Robot.elevator.motionMagicControl();
		
		// if (!Robot.elevator.isAtClimbLimit()) {
			Robot.elevator.setElevator(ControlMode.PercentOutput, signal);
		// } else {
		// 	Robot.elevator.setElevator(ControlMode.PercentOutput, 0);
		// }

		// if (Robot.elevator.isDoneSUCCing()) {
		// 	SmartDashboard.putBoolean("DONE SUCC",true);
		// }
		// System.out.println("Elevator Velocity" +
		// Robot.elevator.elevatorLead.getSelectedSensorVelocity(0));
	}

	
	protected boolean isFinished() {
		return Math.abs(Robot.oi.getOperatorGamepad().getLeftYAxis()) < 0.1;
	}

	
	protected void end() {
		Robot.elevator.setElevator(ControlMode.Position, Robot.elevator.getCurrentPosition(),
				DemandType.ArbitraryFeedForward, Robot.elevator.getArbitraryFeedForward());
	}

	
}
