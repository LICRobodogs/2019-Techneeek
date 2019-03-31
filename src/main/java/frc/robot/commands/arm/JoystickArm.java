package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmControlMode;
import frc.robot.subsystems.Arm.ArmPistonState;

public class JoystickArm extends Command {
    private double signal = 0;

    @Override
    protected boolean isFinished() {
        return Math.abs(Robot.oi.getOperatorGamepad().getRightYAxis()) < 0.25;
    }

    protected void end() {
        Robot.arm.setArmPiston(ArmPistonState.BRAKE);
    }

    public JoystickArm() {
        requires(Robot.arm);
    }

    public void initialize() {
        if (Math.abs(Robot.oi.getOperatorGamepad().getRightYAxis()) > 0.15) {
            Robot.arm.setArmPiston(ArmPistonState.RELEASE);
        }
    }

    @Override
    protected void execute() {
        if (Math.abs(Robot.oi.getOperatorGamepad().getRightYAxis()) > 0.15) {
            signal = Robot.oi.getOperatorGamepad().getRightYAxis();
        } else if (Math.abs(Robot.oi.getOperatorGamepad().getRightYAxis()) < 0.15) {
            signal = -Robot.oi.getOperatorGamepad().getRightYAxis();
        } else {
            signal = 0.0;
        }
        // }
        // SmartDashboard.putNumber("signal", signal);
        // // Robot.elevator.incrementTargetPosition((int) (signal *
        // positionIncrement));
        // // Robot.elevator.motionMagicControl();
        if (Math.abs(Robot.oi.getOperatorGamepad().getRightYAxis()) > 0.15) {
            Robot.arm.setArmAngle(ArmControlMode.MANUAL, signal);
        }
    }
}
