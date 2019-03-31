package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmSide;

/**
 *
 */
public class ArmToggleBack extends InstantCommand {

    public ArmToggleBack() {
        requires(Robot.arm);

    }

    protected void initialize() {
        Robot.arm.setDesiredArmSide(ArmSide.BACK);
        // System.out.println("Switching to Back");
    }

    protected void end() {
        Robot.arm.setHasMoved(false);
    }

}
