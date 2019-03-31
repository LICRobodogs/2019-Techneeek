package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.LimeLight.LED;

/**
 * Turn LED Onn
 */
public class OnLED extends Command {
    
    protected void initialize() {
        Robot.limeLight.setLEID(LED.ON);
    }

    
    protected boolean isFinished() {
        return true;
    }
}
