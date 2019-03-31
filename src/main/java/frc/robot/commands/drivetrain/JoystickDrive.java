package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class JoystickDrive extends Command {

    @Override
    protected boolean isFinished() {
        return false;
    }

    public JoystickDrive() {
        requires(Robot.driveTrain);
    }

    public void execute() {
        double move = 0.75 * Robot.oi.getMoveInput();
        double steer = 0.75 * Robot.oi.getSteerInput();

        Robot.driveTrain.drive(move, -steer);
    }

}
