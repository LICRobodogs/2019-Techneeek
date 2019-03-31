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

    protected void initialize() {

        System.out.println("Trying to zero arm");
        // Robot.arm.resetArmEncoderCertainly();
    }

    protected void execute() {
        // System.out.println("Trying to zero arm");
    }

    protected boolean isFinished() {
        // Robot.arm.getCurrentCommand()
        return true;
    }

}
