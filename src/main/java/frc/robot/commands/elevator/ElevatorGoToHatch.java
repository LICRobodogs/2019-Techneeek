package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmSide;
import frc.util.Constants;

/**
 *
 */
public class ElevatorGoToHatch extends Command {

	private int hatchLevel = 0;

	public ElevatorGoToHatch(int level) {
		requires(Robot.elevator);
		hatchLevel = Robot.elevator.getHatchPosition(level);
	}

	
	protected void initialize() {
		super.setTimeout(5);
		if (hatchLevel == Constants.HATCH_LEVEL1_SETPOINT) {
			if (Robot.arm.getSide() == ArmSide.BACK) {
				end();
			}
		}
		Robot.elevator.setTargetPosition(hatchLevel);
	}

	
	protected void execute() {
		Robot.elevator.motionMagicControl();
	}

	
	protected boolean isFinished() {
		return Robot.elevator.isInPosition(hatchLevel);
	}
}
