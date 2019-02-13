package frc.robot.commands;

import frc.robot.*;
import frc.robot.subsystems.Arm.ArmControlMode;
import edu.wpi.first.wpilibj.command.Command;

public class JoystickArm extends Command {
	private double signal = 0;

	@Override
    protected boolean isFinished() {
        return false;
    }

    public JoystickArm(){
        requires(Robot.arm);
    }
    

    public void initialize(){
        
    }

    @Override
    protected void execute() {
        if(Robot.oi.getOperatorGamepad().getRightYAxis()>0){
			signal = -0.3*Robot.oi.getOperatorGamepad().getRightYAxis();
		}else{
			signal = -0.95*Robot.oi.getOperatorGamepad().getRightYAxis();

		}
		// Robot.elevator.incrementTargetPosition((int) (signal * positionIncrement));
		// Robot.elevator.motionMagicControl();
		Robot.arm.setArmAngle(ArmControlMode.MANUAL, signal);
    }

}
