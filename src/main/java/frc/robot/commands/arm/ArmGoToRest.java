package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmPistonState;

/**
 *
 */
public class ArmGoToRest extends Command {

    private int frontRestPosition = Robot.arm.getFrontRestPosition();
    private boolean allowedToMove = false;

    public ArmGoToRest() {
        requires(Robot.arm);

    }

    protected void initialize() {
        super.setTimeout(5);
        allowedToMove = Robot.arm.setTargetPosition(frontRestPosition);// Robot.wrist.getUpwardLimit() <
        // Robot.wrist.homePosition;

        if (allowedToMove) {
            // System.out.println("Allowed to move to rest");
            Robot.arm.setArmPiston(ArmPistonState.RELEASE);
        } else {
            // System.out.println("Not allowed to move to rest");
        }
    }

    protected void execute() {
        if (allowedToMove) {
            Robot.arm.motionMagicControl();
        }
    }

    protected boolean isFinished() {
        if (allowedToMove) {
            return Robot.arm.isInPosition(frontRestPosition);
        } else {
            return true;
        }
    }

    protected void end() {
        Robot.arm.setHasMoved(true);
        Robot.arm.setArmPiston(ArmPistonState.BRAKE);
    }

}
