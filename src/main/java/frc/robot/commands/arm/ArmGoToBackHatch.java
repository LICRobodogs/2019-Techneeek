package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmPistonState;

/**
 *
 */
public class ArmGoToBackHatch extends Command {

    private int backHatchPosition = Robot.arm.getBackHatchPosition();
    private boolean allowedToMove = true;

    public ArmGoToBackHatch() {
        requires(Robot.arm);
    }

    protected void initialize() {
        super.setTimeout(5);
        Robot.arm.setTargetPosition(backHatchPosition);// Robot.wrist.getUpwardLimit() <
        // Robot.wrist.homePosition;

        if (allowedToMove) {
            // System.out.println("Allowed to move");
            Robot.arm.setArmPiston(ArmPistonState.RELEASE);
        } else {
            // System.out.println("Not allowed to move");
        }
    }

    protected void execute() {
        if (allowedToMove) {
            System.out.println("Going to back hatch");
            Robot.arm.motionMagicControl();
        }
    }

    protected boolean isFinished() {
        if (allowedToMove) {
            return Robot.arm.isInPosition(backHatchPosition);
        } else {
            return true;
        }
    }

    protected void end() {
        Robot.arm.setHasMoved(true);
        Robot.arm.setArmPiston(ArmPistonState.BRAKE);
    }

}
