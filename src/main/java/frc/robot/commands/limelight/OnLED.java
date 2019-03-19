package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmSide;
import frc.robot.subsystems.LimeLight.LED;

/**
 * Turn LED Onn
 */
public class OnLED extends ConditionalCommand {
    @Override
    protected boolean condition() {
            return Robot.arm.getDesiredSide() == ArmSide.FRONT && Robot.arm.getSide() == ArmSide.BACK;
        }
    
    public OnLED() {
        super(new OnLED(), new OffLED());
        requires(Robot.limeLight);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        Robot.limeLight.setLEID(LED.ON);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }
}
