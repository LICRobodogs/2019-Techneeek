package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmControlMode;
import frc.robot.subsystems.Arm.ArmPistonState;

/**
 *
 */
public class KillArm extends Command {

    public KillArm() {
        requires(Robot.arm);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        super.setTimeout(5);
        // Robot.wrist.homePosition;
    }
    
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        Robot.arm.setArmAngle(ArmControlMode.MANUAL,0);// Robot.wrist.getUpwardLimit() <
    }
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
        System.out.println("~~~ARM EMERGENCY STOPPED~~~");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
