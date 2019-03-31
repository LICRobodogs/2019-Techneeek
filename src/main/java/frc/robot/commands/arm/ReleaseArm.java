package frc.robot.commands.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.ArmPistonState;

/**
 *
 */
public class ReleaseArm extends Command {

    public ReleaseArm() {
        requires(Robot.arm);
    }

    protected void initialize() {
        super.setTimeout(5);
        Robot.arm.getMasterTalon().set(ControlMode.PercentOutput, 0);
        Robot.arm.setArmPiston(ArmPistonState.RELEASE);
    }

    protected boolean isFinished() {
        return true;
    }

}
