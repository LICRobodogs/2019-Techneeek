package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmSide;
import frc.util.Constants;

/**
 *
 */
public class ElevatorGoToCargo extends Command {

	private int cargoLevel = 0;

	public ElevatorGoToCargo(int level) {
		requires(Robot.elevator);
		cargoLevel = Robot.elevator.getCargoPosition(level);
	}

	
	protected void initialize() {
		super.setTimeout(5);
		if (cargoLevel == Constants.CARGO_LEVEL1_SETPOINT) {
			if (Robot.arm.getSide() == ArmSide.BACK) {
				end();
			}
		}
		Robot.elevator.setTargetPosition(cargoLevel);
	}

	
	protected void execute() {
		Robot.elevator.motionMagicControl();
	}

	
	protected boolean isFinished() {
		return Robot.elevator.isInPosition(cargoLevel);
	}

}
