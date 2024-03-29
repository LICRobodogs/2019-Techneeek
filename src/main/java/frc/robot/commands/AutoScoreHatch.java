package frc.robot.commands;

import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmSide;
import frc.robot.subsystems.LimeLight.LED;

/**
 * Turn LED On if 3d compute is available
 */
public class AutoScoreHatch extends ConditionalCommand {
    @Override
    protected boolean condition() {
        return Robot.limeLight.is3dCompute();
    }
    
    public AutoScoreHatch() {
        super(new DriveAndScoreHatch());
        requires(Robot.limeLight);
    }
}
