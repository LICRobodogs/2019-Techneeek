package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class GoStraightAtPercent extends Command {
    private double percent;
	@Override
    protected boolean isFinished() {
        return false;
    }

    public GoStraightAtPercent(double percent){
        this.percent = percent;
        requires(Robot.driveBaseSubsystem);
    }
    

    public void initialize(){        
    }

    public void execute() {
        // Robot.driveBaseSubsystem.drive(Robot.getPsController().xSpeed(), Robot.getPsController().zRotation());
        
        Robot.driveBaseSubsystem.setPercentSpeed(percent);
    }

}
