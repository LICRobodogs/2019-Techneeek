package frc.robot.commands.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmPistonState;

/**
 *
 */
public class ElevatorGoToIntake extends Command {

	public ElevatorGoToIntake() {
		requires(Robot.elevator);
	}

	
	protected void initialize() {
		super.setTimeout(5);
		Robot.elevator.setTargetPosition(Robot.elevator.getCollectPosition());
	}

	
	protected void execute() {
		Robot.elevator.motionMagicControl();
	}

	
	protected boolean isFinished() {
		return Robot.elevator.isInPosition(Robot.elevator.getCollectPosition());
	}

	protected void end() {
		Robot.arm.getMasterTalon().set(ControlMode.PercentOutput, 0);
		Robot.arm.setArmPiston(ArmPistonState.RELEASE);
	}

}
