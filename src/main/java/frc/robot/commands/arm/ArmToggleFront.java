package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmSide;

/**
 *
 */
public class ArmToggleFront extends InstantCommand {

    public ArmToggleFront() {
        requires(Robot.arm);
    }

    protected void initialize() {
        Robot.arm.setDesiredArmSide(ArmSide.FRONT);
        // System.out.println("Switching to Front");
    }

    protected void end() {
        Robot.arm.setHasMoved(false);
    }

}
