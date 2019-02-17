package frc.robot.commands.arm;

import frc.robot.*;
import frc.robot.subsystems.Arm.ArmControlMode;
import frc.robot.subsystems.Arm.ArmPistonState;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class JoystickArm extends Command {
	private double signal = 0;

	@Override
    protected boolean isFinished() {
        return true;
    }

    protected void end(){
        Robot.arm.setArmPiston(ArmPistonState.BRAKE);
    }

    public JoystickArm(){
        requires(Robot.arm);
    }
    

    public void initialize(){
        Robot.arm.setArmPiston(ArmPistonState.RELEASE);
    }

    @Override
    protected void execute() {
        if(Robot.oi.getOperatorGamepad().getRightYAxis()>0){
			signal = -0.3*Robot.oi.getOperatorGamepad().getRightYAxis();
		}else{
			signal = -0.95*Robot.oi.getOperatorGamepad().getRightYAxis();

        }
        SmartDashboard.putNumber("signal", signal);
		// Robot.elevator.incrementTargetPosition((int) (signal * positionIncrement));
        // Robot.elevator.motionMagicControl();
        if(Robot.arm.isValidPosition(signal,true)){
            Robot.arm.setArmAngle(ArmControlMode.MANUAL, signal);
        }
    }

}
