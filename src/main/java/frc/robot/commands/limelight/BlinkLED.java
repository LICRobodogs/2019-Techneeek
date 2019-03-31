package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.LimeLight.LED;

/**
 * Starts Blinking to LED
 */
public class BlinkLED extends Command {
    
    protected void initialize() {
        Robot.limeLight.setLEID(LED.BLINK);
    }

    
    protected boolean isFinished() {
        return true;
    }
}
