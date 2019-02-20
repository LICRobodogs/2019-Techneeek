package frc.robot.commands;

import frc.robot.*;

import edu.wpi.first.wpilibj.command.Command;

public class SetDriveSpeedSlow extends Command {

	@Override
    protected boolean isFinished() {
        return true;
    }

    public SetDriveSpeedSlow(){
        requires(Robot.driveTrain);
    }
    

    public void initialize(){
        Robot.driveTrain.setDesiredSpeed(0.35);
    }

    public void execute() {

    }

}
