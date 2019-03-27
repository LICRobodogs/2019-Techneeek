package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;



/**
 *
 */
public class ZeroArm extends Command {

    public ZeroArm() {
        requires(Robot.arm);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        
        System.out.println("Trying to zero arm");
        // Robot.arm.resetArmEncoderCertainly();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        // System.out.println("Trying to zero arm");
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        // Robot.arm.getCurrentCommand()
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
