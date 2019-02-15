package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.controller.Ps4_Controller;
import frc.robot.Robot;
import frc.util.drivers.Controllers;

public class JoystickDrive extends Command {
    private static Ps4_Controller ps;
	@Override
    protected boolean isFinished() {
        return false;
    }

    public JoystickDrive(){
        requires(Robot.driveBaseSubsystem);
        ps = Controllers.getInstance().getPS_Controller();
    }
    

    public void initialize(){        
    }

    public void execute() {
        // Robot.driveBaseSubsystem.setSpeed(Robot.getPsController().xSpeed(), Robot.getPsController().zRotation());
        Robot.driveBaseSubsystem.drive(ps.xSpeed(), ps.zRotation());
    }

}
