package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmControlMode;

/**
 *
 */
public class KillArm extends Command {

    public KillArm() {
        requires(Robot.arm);
    }

    protected void initialize() {
        super.setTimeout(5);
        // Robot.wrist.homePosition;
    }

    protected void execute() {
        Robot.arm.setArmAngle(ArmControlMode.MANUAL, 0);// Robot.wrist.getUpwardLimit() <
    }

    protected boolean isFinished() {
        return false;
    }

    protected void end() {
        System.out.println("~~~ARM EMERGENCY STOPPED~~~");
    }

}
