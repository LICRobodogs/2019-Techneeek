
package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;

/**
 *
 */
public class ZeroElevator extends Command {
    public ZeroElevator() {
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        Robot.elevator.elevatorLead.setSelectedSensorPosition(4000);
        
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        Robot.elevator.elevatorLead.setSelectedSensorPosition(4000);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
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
