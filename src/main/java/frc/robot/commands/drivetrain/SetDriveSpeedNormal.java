package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class SetDriveSpeedNormal extends Command {

    @Override
    protected boolean isFinished() {
        return true;
    }

    public SetDriveSpeedNormal() {
        requires(Robot.driveTrain);
    }

    public void initialize() {
        Robot.driveTrain.setDesiredSpeed(0.75);
    }

}
