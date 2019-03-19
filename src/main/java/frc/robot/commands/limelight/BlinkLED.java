package frc.robot.commands.limelight;

import frc.robot.*;
import frc.robot.subsystems.LimeLight.LED;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Starts Blinking to LED
 */
public class BlinkLED extends Command {
    public BlinkLED() {
        // requires(Robot.limeLight);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void initialize() {
        Robot.limeLight.setLEID(LED.BLINK);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }
}
