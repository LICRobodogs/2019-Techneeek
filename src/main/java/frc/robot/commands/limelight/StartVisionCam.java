package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.LimeLight.CameraMode;

/**
 * Starts Blinking to LED
 */
public class StartVisionCam extends Command {
    
    protected void initialize() {
        Robot.limeLight.switchCameraMode(CameraMode.VISION_PROCESSOR);
    }

    
    protected boolean isFinished() {
        return true;
    }
}
