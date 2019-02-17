package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmSide;

/**
 *
 */
public class ArmToggleFront extends InstantCommand {

    public ArmToggleFront() {
        requires(Robot.arm);

    }

    // Called just before this Command runs the first time
    protected void initialize() {
        Robot.arm.setDesiredArmSide(ArmSide.FRONT);
        // System.out.println("Switching to Front");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.arm.setHasMoved(false);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
