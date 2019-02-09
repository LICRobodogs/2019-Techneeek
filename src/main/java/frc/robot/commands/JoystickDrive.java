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
        requires(DriveBaseSubsystem.getInstance());
    }
    

    public void initialize(){
        
    }

    public void execute() {
        double move = Robot.oi.getMoveInput();
        double steer = Robot.oi.getSteerInput();
        DriveBaseSubsystem.getInstance().drive(move,steer);
    }

}
