package frc.robot.commands.misc;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Turn LED Off
 */
public class UseDashBoardVariable extends Command {
    public double varData;

    public UseDashBoardVariable() {

        // requires(Robot.limeLight);
    }

    
    protected void initialize() {
        System.out.println("THE COMMAND STARTED");
        varData = SmartDashboard.getNumber("Arm Angle: ", -1);
    }

    
    protected boolean isFinished() {
        return true;
    }

    protected void end() {
        System.out.println("DA DATA" + varData);
    }
}
