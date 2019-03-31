package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmPistonState;

/**
 *
 */
public class HoldArm extends Command {

  private int currentPosition = Robot.arm.getCurrentPosition();

  public HoldArm() {
    requires(Robot.arm);
  }

  protected void initialize() {
    super.setTimeout(5);
    Robot.arm.setTargetPosition(currentPosition);// Robot.wrist.getUpwardLimit() <
    // Robot.wrist.homePosition;
  }

  protected void execute() {
    Robot.arm.motionMagicControl();
  }

  protected boolean isFinished() {
    return Robot.arm.isInPosition(currentPosition);
  }

  protected void end() {
    Robot.arm.setArmPiston(ArmPistonState.BRAKE);
    System.out.println("~~~ARM IS HOLDING~~~");
  }

}
