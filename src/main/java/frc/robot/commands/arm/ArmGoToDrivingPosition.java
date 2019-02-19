package frc.robot.commands.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmPistonState;
import frc.robot.subsystems.Arm.ArmSide;

/**
 *
 */
public class ArmGoToDrivingPosition extends Command {

    private int drivingPosition = Robot.arm.getDrivingPosition();
    private boolean allowedToMove = false;

    public ArmGoToDrivingPosition() {
        // super()
        requires(Robot.arm);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        setTimeout(3);
        allowedToMove = Robot.arm.setTargetPosition(drivingPosition);// Robot.wrist.getUpwardLimit() <
        // Robot.wrist.homePosition;

        if (allowedToMove) {
            // System.out.println("Allowed to move");
            Robot.arm.setArmPiston(ArmPistonState.RELEASE);
    } else {
            // System.out.println("Not allowed to move");
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
			return Robot.arm.isInPosition(drivingPosition);
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
