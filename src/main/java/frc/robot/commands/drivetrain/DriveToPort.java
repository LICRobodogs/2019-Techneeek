package frc.robot.commands.drivetrain;

import frc.robot.*;
import frc.robot.subsystems.LimeLight.TargetType;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Command to drive to a port
 */
public class DriveToPort extends Command {
    public DriveToPort() {
        requires(Robot.driveTrain);
        requires(Robot.limeLight);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        Robot.limeLight.getInPortRange();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.limeLight.isAtTarget(TargetType.PORT);
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
