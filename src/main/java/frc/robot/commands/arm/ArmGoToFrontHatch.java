package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmPistonState;

/**
 *
 */
public class ArmGoToFrontHatch extends Command {

    private int frontHatchPosition = Robot.arm.getFrontHatchPosition();
    private boolean allowedToMove = false;

    public ArmGoToFrontHatch() {
        requires(Robot.arm);
    }

    protected void initialize() {
        super.setTimeout(5);
        allowedToMove = Robot.arm.setTargetPosition(frontHatchPosition);// Robot.wrist.getUpwardLimit() <
        // Robot.wrist.homePosition;

        if (allowedToMove) {
            System.out.println("Allowed to move to front hatch");
            Robot.arm.setArmPiston(ArmPistonState.RELEASE);
        } else {
            System.out.println("Not allowed to move");
        }
    }

    protected void execute() {
        if (allowedToMove) {
            Robot.arm.motionMagicControl();
        }
    }

    protected boolean isFinished() {
        if (allowedToMove) {
            return Robot.arm.isInPosition(frontHatchPosition);
        } else {
            return true;
        }
    }

    protected void end() {
        Robot.arm.setHasMoved(true);
        Robot.arm.setArmPiston(ArmPistonState.BRAKE);
        System.out.println("~~~ARM IS AT FRONT HATCH~~~");
    }

}
