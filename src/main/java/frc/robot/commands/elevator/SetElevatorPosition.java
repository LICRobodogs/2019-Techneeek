package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * Turn LED Off
 */
public class SetElevatorPosition extends Command {
    public double varData;

    public SetElevatorPosition() {
        requires(Robot.elevator);
    }

    
    protected void initialize() {
        System.out.println("THE COMMAND STARTED");
        varData = SmartDashboard.getNumber("ElevatorVariable",0.0);
        Robot.elevator.setElevatorPosition((int)varData);
    }

    
    protected boolean isFinished() {
        return true;
    }

    protected void end() {
        System.out.println("DA DATA" + varData);
    }
}
