package frc.robot.commands;

import frc.robot.*;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class PreventElevatorCollision extends Command {

    boolean isArmSafe = false;
    boolean isElevatorSafe = false;
    private int restPosition = Robot.arm.getFrontRestPosition();
    private int desiredArmPosition = restPosition;

	public PreventElevatorCollision(int desiredArmPosition) {
		// Use requires() here to declare subsystem dependencies
        requires(Robot.arm);
		requires(Robot.elevator);
        this.desiredArmPosition = desiredArmPosition;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		int armPosition = Robot.arm.getCurrentPosition();
		int elevatorPosition = Robot.elevator.getCurrentPosition();
		System.out.println("Arm position: " + armPosition);
		System.out.println("Elevator position: " + elevatorPosition);
        if (elevatorPosition < Robot.elevator.getTopOfFirstStagePosition() && (Robot.arm.getCurrentPosition()>Robot.arm.getSafePosition() && Robot.arm.getSafePosition()>this.desiredArmPosition)) {
            isElevatorSafe = false;
			Robot.elevator.setTargetPosition(Robot.elevator.getTopOfFirstStagePosition());
		} else {
            isElevatorSafe = true;
		}
		System.out.println("Elevator is safe: " + isElevatorSafe);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		int elevatorPosition = Robot.elevator.getCurrentPosition();
        if (!isElevatorSafe) {
            System.out.println("moving elevator");
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
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
