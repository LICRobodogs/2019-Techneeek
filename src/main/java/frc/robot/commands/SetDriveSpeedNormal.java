package frc.robot.commands;

import frc.robot.*;

import edu.wpi.first.wpilibj.command.Command;

public class SetDriveSpeedNormal extends Command {

	@Override
    protected boolean isFinished() {
        return true;
    }

    public SetDriveSpeedNormal(){
        requires(Robot.driveTrain);
    }
    

    public void initialize(){
        Robot.driveTrain.setDesiredSpeed(0.4);
    }

    public void execute() {

    }

}
