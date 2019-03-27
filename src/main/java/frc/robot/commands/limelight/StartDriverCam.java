package frc.robot.commands.limelight;

import frc.robot.*;
import frc.robot.subsystems.LimeLight.LED;
import frc.robot.subsystems.LimeLight.CameraMode;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Starts Blinking to LED
 */
public class StartDriverCam extends Command {
    public StartDriverCam() {
        // requires(Robot.limeLight);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void initialize() {
        Robot.limeLight.switchCameraMode(CameraMode.DRIVER_CAMERA);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }
}
