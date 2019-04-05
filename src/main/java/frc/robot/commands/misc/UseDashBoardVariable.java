package frc.robot.commands.misc;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * Turn LED Off
 */
public class UseDashBoardVariable extends Command {
    public double varData;

    public UseDashBoardVariable() {
        requires(Robot.arm);
    }

    
    protected void initialize() {
        System.out.println("THE COMMAND STARTED");
        varData = SmartDashboard.getNumber("ChangeThisVariable",0.0);
        Robot.arm.setStartConfigAngle((int)varData);
    }

    
    protected boolean isFinished() {
        return true;
    }

    protected void end() {
        System.out.println("DA DATA" + varData);
    }
}
