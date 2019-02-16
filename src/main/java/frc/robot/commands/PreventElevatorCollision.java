package frc.robot.commands;

import frc.robot.*;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class PreventElevatorCollision extends Command {

    boolean isElevatorSafe = false;
	Command nextCommand = new WaitCommand("placeholder command", 2);

	public PreventElevatorCollision() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.elevator);
		nextCommand = new WaitCommand("blank constructor waitcommand", 0);
	}

	public PreventElevatorCollision(Command next) {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.elevator);
		nextCommand = next;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.elevator.setTargetPosition(Robot.elevator.getTopOfFirstStagePosition());
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		int elevatorPosition = Robot.elevator.getCurrentPosition();
        if (!isElevatorSafe) {
            System.out.println("moving elevator to prevent collision");
            Robot.elevator.motionMagicControl();
            if(elevatorPosition >= Robot.elevator.getTopOfFirstStagePosition()){
                isElevatorSafe = true;
		        System.out.println("Elevator is safe: " + isElevatorSafe);
            }
        }
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		boolean beyondTarget = Math.abs(Robot.elevator.getCurrentPosition() - Robot.elevator.getTopOfFirstStagePosition()) <= Robot.elevator.getTargetThreshold();;
        return isElevatorSafe || beyondTarget;
	}

	// Called once after isFinished returns true
	protected void end() {
		nextCommand.start();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
