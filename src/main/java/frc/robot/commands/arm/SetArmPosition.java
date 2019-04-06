package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * Turn LED Off
 */
public class SetArmPosition extends Command {
    public double varData;

    public SetArmPosition() {
        requires(Robot.arm);
    }

    
    protected void initialize() {
        System.out.println("THE COMMAND STARTED");
        varData = SmartDashboard.getNumber("ArmVariable",0.0);
        Robot.arm.setStartConfigAngle((int)varData);
    }

    
    protected boolean isFinished() {
        return true;
    }

    protected void end() {
        System.out.println("DA DATA" + varData);
    }
}
