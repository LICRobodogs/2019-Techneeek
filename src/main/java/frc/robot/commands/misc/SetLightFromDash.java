package frc.robot.commands.misc;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.util.Constants;

/**
 * Turn LED Off
 */
public class SetLightFromDash extends Command {
    public double varData;

    public SetLightFromDash() {
    }

    
    protected void initialize() {
        varData = SmartDashboard.getNumber("KPSTEER",0.0);
        Constants.KpDrive = SmartDashboard.getNumber("KPDRIVE",0.0);
        Constants.KpSteer = SmartDashboard.getNumber("KPSTEER",0.0);
    }

    
    protected boolean isFinished() {
        return true;
    }

    protected void end() {
        System.out.println("DA DATA" + varData);
    }
}
