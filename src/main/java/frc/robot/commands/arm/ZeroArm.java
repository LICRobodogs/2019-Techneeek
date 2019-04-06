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
        System.out.println("THE COMMAND STARTED");
        Robot.arm.setStartConfigAngle(0);
    }

    
    protected boolean isFinished() {
        return true;
    }

}
