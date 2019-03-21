package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmPistonState;

/**
 *
 */
public class HoldArm extends Command {

    private int currentPosition = Robot.arm.getCurrentPosition();

    public HoldArm() {
        requires(Robot.arm);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        super.setTimeout(5);
        Robot.arm.setTargetPosition(currentPosition);// Robot.wrist.getUpwardLimit() <
        // Robot.wrist.homePosition;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
		Robot.arm.motionMagicControl();
    }
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
		return Robot.arm.isInPosition(currentPosition);
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.arm.setArmPiston(ArmPistonState.BRAKE);
        System.out.println("~~~ARM IS HOLDING~~~");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
