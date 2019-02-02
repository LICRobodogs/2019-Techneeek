package frc.robot.commands;

import frc.robot.*;

import edu.wpi.first.wpilibj.command.Command;

public class JoystickArm extends Command {

	@Override
    protected boolean isFinished() {
        return false;
    }

    public JoystickArm(){
        requires(Robot.arm);
    }
    

    public void initialize(){
        
    }

    public void execute() {
       Robot.arm.moveWithJoystick();
    }

}
