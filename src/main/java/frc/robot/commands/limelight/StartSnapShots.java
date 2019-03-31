package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Start Taking Pictures
 */
public class StartSnapShots extends Command {
    
    protected void initialize() {
        Robot.limeLight.startTakingSnapshots();
    }

    
    protected boolean isFinished() {
        return true;
    }
}
