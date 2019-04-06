
package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 *
 */
public class ZeroElevator extends Command {
    public ZeroElevator() {
        requires(Robot.elevator);
    }

    
    protected void initialize() {
        Robot.elevator.zeroElevator();

    }
    
    protected boolean isFinished() {
        return true;
    }

}
