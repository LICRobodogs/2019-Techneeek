package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 *
 */
public class PreventArmCollision extends Command {

	boolean isArmSafe = false;
	boolean isElevatorSafe = false;
	private int safePosition = Robot.arm.getSafePosition();
	private int restPosition = Robot.arm.getFrontRestPosition();
	private int desiredArmPosition = restPosition;

	public PreventArmCollision(int desiredArmPosition) {

		requires(Robot.arm);
		requires(Robot.elevator);
		this.desiredArmPosition = desiredArmPosition;
	}

	protected void initialize() {
		int armPosition = Robot.arm.getCurrentPosition();
		int elevatorPosition = Robot.elevator.getCurrentPosition();
		// System.out.println("Arm position arm: " + armPosition);
		// System.out.println("Elevator position elevator: " + elevatorPosition);
		if (((desiredArmPosition > safePosition && armPosition < safePosition)
				|| (desiredArmPosition < safePosition && armPosition > safePosition))
				&& elevatorPosition < Robot.elevator.getTopOfFirstStagePosition()) {
			isArmSafe = false;
			isElevatorSafe = false;
			Robot.arm.setTargetPosition(restPosition);
			Robot.elevator.setTargetPosition(Robot.elevator.getTopOfFirstStagePosition());
		} else if (((desiredArmPosition > safePosition && armPosition < safePosition)
				|| (desiredArmPosition < safePosition && armPosition > safePosition))) {
			isArmSafe = false;
			isElevatorSafe = true;
			Robot.arm.setTargetPosition(restPosition);
		} else {
			isArmSafe = true;
			isElevatorSafe = true;
		}
		// System.out.println("Arm is safe arm: " + isArmSafe);
		// System.out.println("Elevator is safe elevator: " + isElevatorSafe);
	}

	protected void execute() {
		int elevatorPosition = Robot.elevator.getCurrentPosition();
		if (!isArmSafe && isElevatorSafe) {
			Robot.arm.motionMagicControl();
			// System.out.println("moving arm");
		}
		if (!isElevatorSafe) {
			// System.out.println("moving elevator in arm collision command");
			// Robot.elevator.motionMagicControl();
			if (elevatorPosition >= Robot.elevator.getTopOfFirstStagePosition()) {
				isElevatorSafe = true;
				// System.out.println("Elevator is safe and done: " + isElevatorSafe);
			}
		}
	}

	protected boolean isFinished() {
		boolean beyondTarget = Math.abs(Robot.arm.getCurrentPosition() - restPosition) <= Robot.arm
				.getTargetThreshold();
		return isArmSafe || beyondTarget;
	}

}
