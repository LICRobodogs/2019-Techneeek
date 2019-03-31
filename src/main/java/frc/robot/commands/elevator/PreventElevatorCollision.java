package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.Robot;

/**
 *
 */
public class PreventElevatorCollision extends Command {

	boolean isElevatorSafe = false;
	Command nextCommand = new WaitCommand("placeholder command", 2);

	public PreventElevatorCollision() {

		requires(Robot.elevator);
		nextCommand = new WaitCommand("blank constructor waitcommand", 0);
	}

	public PreventElevatorCollision(Command next) {

		requires(Robot.elevator);
		this.nextCommand = next;
	}

	
	protected void initialize() {
		super.setTimeout(5);
		Robot.elevator.setTargetPosition(Robot.elevator.getDunkPosition());
	}

	
	protected void execute() {
		int elevatorPosition = Robot.elevator.getCurrentPosition();
		if (!isElevatorSafe) {
			// System.out.println("moving elevator to prevent collision");
			Robot.elevator.motionMagicControl();
			if (elevatorPosition >= Robot.elevator.getTopOfFirstStagePosition()) {
				isElevatorSafe = true;
				// System.out.println("Elevator is safe: " + isElevatorSafe);
			}
		}
	}

	
	protected boolean isFinished() {
		boolean beyondTarget = Robot.elevator.getCurrentPosition() >= Robot.elevator.getTopOfFirstStagePosition();
		return isElevatorSafe || beyondTarget;
	}

	protected void end() {
		nextCommand.start();
	}

}
