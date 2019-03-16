package frc.robot.commands.drivetrain;

import frc.robot.*;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Spins until a target is found, then centers on it
 */
public class FindTarget extends Command {
    public FindTarget() {
        requires(Robot.driveTrain);
        // requires(Robot.limeLight);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        Robot.limeLight.seekTarget();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.limeLight.isAimed();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
