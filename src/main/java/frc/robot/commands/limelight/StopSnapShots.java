package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Stop taking snapshots
 */
public class StopSnapShots extends Command {

    
    protected void initialize() {
        Robot.limeLight.stopTakingSnapshots();
    }

    
    protected boolean isFinished() {
        return true;
    }
}
