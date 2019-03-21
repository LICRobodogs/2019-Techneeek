package frc.robot.commands.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmPistonState;

/**
 *
 */
public class ReleaseArm extends Command {

    public ReleaseArm() {
        requires(Robot.arm);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        super.setTimeout(5);
        Robot.arm.getMasterTalon().set(ControlMode.PercentOutput,0);
        Robot.arm.setArmPiston(ArmPistonState.RELEASE);
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

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
