
package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 *
 */
public class ZeroElevator extends Command {

    
    protected void initialize() {
        Robot.elevator.elevatorLead.setSelectedSensorPosition(4000);

    }

    
    protected void execute() {
        Robot.elevator.elevatorLead.setSelectedSensorPosition(4000);
    }

    
    protected boolean isFinished() {
        return true;
    }

}
