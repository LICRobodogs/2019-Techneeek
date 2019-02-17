package frc.robot.commands.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmSide;

/**
 *
 */
public class ArmGoToHome extends Command {

    private int collect = Robot.arm.getCollectPosition();
    private boolean allowedToMove = false;

    public ArmGoToHome() {
        requires(Robot.arm);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        allowedToMove = Robot.arm.setTargetPosition(collect);// Robot.wrist.getUpwardLimit() <
        // Robot.wrist.homePosition;

        if (allowedToMove) {
            System.out.println("Allowed to move to home");
        } else {
            System.out.println("Not allowed to move to home");
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
			return Robot.arm.isInPosition(collect);
		} else {
			return true;
		}    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.arm.setHasMoved(true);
        Robot.arm.setArmSide(ArmSide.SAME);
        Robot.arm.getMasterTalon().set(ControlMode.PercentOutput,0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}