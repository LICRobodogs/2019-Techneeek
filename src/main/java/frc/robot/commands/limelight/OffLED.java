package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.LimeLight.LED;

/**
 * Turn LED Off
 */
public class OffLED extends Command {
    
    protected void initialize() {
        Robot.limeLight.setLEID(LED.OFF);
    }

    
    protected boolean isFinished() {
        return true;
    }
}
