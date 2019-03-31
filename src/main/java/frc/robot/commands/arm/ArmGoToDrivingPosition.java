package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmPistonState;

/**
 *
 */
public class ArmGoToDrivingPosition extends Command {

    private int drivingPosition = Robot.arm.getDrivingPosition();
    private boolean allowedToMove = false;

    public ArmGoToDrivingPosition() {
        // super()
        requires(Robot.arm);
    }

    protected void initialize() {
        super.setTimeout(5);
        allowedToMove = Robot.arm.setTargetPosition(drivingPosition);// Robot.wrist.getUpwardLimit() <
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
            Robot.arm.motionMagicControl();
        }
    }

    protected boolean isFinished() {
        if (allowedToMove) {
            return Robot.arm.isInPosition(drivingPosition);
        } else {
            return true;
        }
    }

    protected void end() {
        Robot.arm.setHasMoved(true);
        Robot.arm.setArmPiston(ArmPistonState.BRAKE);
    }

}
