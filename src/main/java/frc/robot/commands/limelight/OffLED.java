package frc.robot.commands.limelight;

import frc.robot.*;
import frc.robot.subsystems.LimeLight.LED;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Turn LED Off
 */
public class OffLED extends Command {
    public OffLED() {
        requires(Robot.limeLight);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        Robot.limeLight.setLEID(LED.OFF);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }
}
