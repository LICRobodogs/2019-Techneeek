package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmPistonState;

/**
 * Puts are in back cargo position
 */
public class ArmGoToBackCargo extends Command {

    private int backCargoPosition = Robot.arm.getBackCargoPosition();
    private boolean allowedToMove = true;

    public ArmGoToBackCargo() {
        requires(Robot.arm);
    }

    protected void initialize() {
        super.setTimeout(5);
        Robot.arm.setTargetPosition(backCargoPosition);// Robot.wrist.getUpwardLimit() <
        // Robot.wrist.homePosition;

        if (allowedToMove) {
            Robot.arm.setArmPiston(ArmPistonState.RELEASE);
        } else {
        }
    }

    protected void execute() {
        if (allowedToMove) {
            System.out.println("Going to back cargo");
            Robot.arm.motionMagicControl();
        }
    }

    protected boolean isFinished() {
        if (allowedToMove) {
            return Robot.arm.isInPosition(backCargoPosition);
        }
        return true;
    }

    protected void end() {
        Robot.arm.setHasMoved(true);
        Robot.arm.setArmPiston(ArmPistonState.BRAKE);
    }
}
