package frc.robot.commands;

import frc.robot.*;

import edu.wpi.first.wpilibj.command.Command;

public class JoystickDrive extends Command {

	@Override
    protected boolean isFinished() {
        return false;
    }

    public JoystickDrive(){
        requires(Robot.driveTrain);
    }
    

    public void initialize(){
        
    }

    public void execute() {
        double move = Robot.oi.getMoveInput();
        double steer = Robot.oi.getSteerInput();

        Robot.driveTrain.drive(move, steer);
    }

}
