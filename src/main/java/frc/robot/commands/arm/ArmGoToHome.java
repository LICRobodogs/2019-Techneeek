package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmPistonState;

/**
 *
 */
public class ArmGoToHome extends Command {

    private int collect = Robot.arm.getCollectPosition();
    private boolean allowedToMove = false;
    private boolean engageBrake = false;

    public ArmGoToHome() {
        requires(Robot.arm);
    }

    public ArmGoToHome(boolean brake) {
        requires(Robot.arm);
        this.engageBrake = brake;
    }

    protected void initialize() {
        super.setTimeout(5);
        allowedToMove = Robot.arm.setTargetPosition(collect);// Robot.wrist.getUpwardLimit() <
        // Robot.wrist.homePosition;

        if (allowedToMove) {
            // System.out.println("Allowed to move to home");
            Robot.arm.setArmPiston(ArmPistonState.RELEASE);
        } else {
            // System.out.println("Not allowed to move to home");
        }
    }

    protected void execute() {
        if (allowedToMove) {
            Robot.arm.motionMagicControl();
        }
    }

    protected boolean isFinished() {
        if (allowedToMove) {
            return Robot.arm.isInPosition(collect);
        } else {
            return true;
        }
    }

    protected void end() {
        Robot.arm.setHasMoved(true);
        Robot.arm.setArmPiston(ArmPistonState.BRAKE);
    }

}
