package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmSide;

/**
 *
 */
public class ArmToggleFront extends Command {

    public ArmToggleFront() {
        requires(Robot.arm);

    }

    // Called just before this Command runs the first time
    protected void initialize() {
        Robot.arm.setArmSide(ArmSide.FRONT);
        System.out.println("Switching to Front");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
            return true;
		} 
			

    // Called once after isFinished returns true
    protected void end() {
        // Robot.elevator.setElevator(ControlMode.Position,
        // Robot.elevator.getCurrentPosition(),DemandType.ArbitraryFeedForward,Robot.elevator.getArbitraryFeedForward());
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
