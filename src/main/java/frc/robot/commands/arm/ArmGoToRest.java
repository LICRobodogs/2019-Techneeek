package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmPistonState;

/**
 *
 */
public class ArmGoToRest extends Command {

    private int frontRestPosition = Robot.arm.getFrontRestPosition();
    private boolean allowedToMove = false;

    public ArmGoToRest() {
        requires(Robot.arm);

    }

    // Called just before this Command runs the first time
    protected void initialize() {
        super.setTimeout(5);
        allowedToMove = Robot.arm.setTargetPosition(frontRestPosition);// Robot.wrist.getUpwardLimit() <
        // Robot.wrist.homePosition;

        if (allowedToMove) {
            // System.out.println("Allowed to move to rest");
            Robot.arm.setArmPiston(ArmPistonState.RELEASE);   
        } else {
            // System.out.println("Not allowed to move to rest");
        }
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        if (allowedToMove) {
			Robot.arm.motionMagicControl();
	    }
    }
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        if (allowedToMove) {
			return Robot.arm.isInPosition(frontRestPosition);
		} else {
			return true;
		}    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.arm.setHasMoved(true);
        Robot.arm.setArmPiston(ArmPistonState.BRAKE);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
