package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.util.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmSide;

/**
 *
 */
public class ElevatorGoToCargo extends Command {

    private int cargoLevel = 0;
    
	public ElevatorGoToCargo(int level) {
		requires(Robot.elevator);
		cargoLevel = Robot.elevator.getCargoPosition(level);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		super.setTimeout(5);
		if (cargoLevel == Constants.CARGO_LEVEL1_SETPOINT) {
			if(Robot.arm.getSide() == ArmSide.BACK) {
				end();
			}
		}
        Robot.elevator.setTargetPosition(cargoLevel);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
        Robot.elevator.motionMagicControl();
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
        return Robot.elevator.isInPosition(cargoLevel);
    }

	// Called once after isFinished returns true
	protected void end() {
		// Robot.elevator.setElevator(ControlMode.Position, Robot.elevator.getCurrentPosition(),DemandType.ArbitraryFeedForward,Robot.elevator.getArbitraryFeedForward());
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
