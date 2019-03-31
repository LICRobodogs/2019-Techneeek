package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.LimeLight.CameraMode;

/**
 * Starts Driver Cam
 */
public class StartDriverCam extends Command {
    
    protected void initialize() {
        Robot.limeLight.switchCameraMode(CameraMode.DRIVER_CAMERA);
    }

    
    protected boolean isFinished() {
        return true;
    }
}
