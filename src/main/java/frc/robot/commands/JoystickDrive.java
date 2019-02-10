package frc.robot.commands;

import frc.robot.*;
import frc.robot.subsystems.DriveBaseSubsystem;
import edu.wpi.first.wpilibj.command.Command;

public class JoystickDrive extends Command {

	@Override
    protected boolean isFinished() {
        return false;
    }

    public JoystickDrive(){
        requires(Robot.driveBaseSubsystem);
    }
    

    public void initialize(){        
    }

    public void execute() {
        // Robot.driveBaseSubsystem.setSpeed(Robot.getPsController().xSpeed(), Robot.getPsController().zRotation());
        Robot.driveBaseSubsystem.drive(Robot.getPsController().xSpeed(), Robot.getPsController().zRotation());
    }

}
