package frc.robot.commands.misc;

import frc.robot.*;
import frc.robot.subsystems.LimeLight.LED;
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

    // Called repeatedly when this Command is scheduled to run
    protected void initialize() {
        System.out.println("THE COMMAND STARTED");
        varData = SmartDashboard.getNumber("Arm Angle: ", -1);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }
    protected void end() {
        System.out.println("DA DATA"+varData);
    }
}
