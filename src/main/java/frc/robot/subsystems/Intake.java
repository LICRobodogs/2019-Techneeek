package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.util.ControlLoopable;
import frc.util.Constants;

public class Intake extends Subsystem implements ControlLoopable {
	private static Intake instance = null;
	public static Intake getInstance() {
		if (instance == null)
			instance = new Intake();
		return instance;
	}
    public static enum IntakeState {
        SUCC_IN, SUCC_OUT
    };

	private IntakeState succState = IntakeState.SUCC_OUT;

	private static Solenoid leftSuctionSolenoid;
	private static Solenoid rightSuctionSolenoid;
    private TalonSRX topIntake, leftSuction, rightSuction;
    private VictorSPX bottomIntake;

	private Intake() {
		try {
			topIntake = new TalonSRX(Constants.TOP_INTAKE_TALON_ID);
            bottomIntake = new VictorSPX(Constants.BOTTOM_INTAKE_VICTOR_ID);
            leftSuction = new TalonSRX(Constants.LEFT_SUCTION_TALON_ID);
            rightSuction = new TalonSRX(Constants.RIGHT_SUCTION_TALON_ID);
            leftSuctionSolenoid = new Solenoid(Constants.LEFT_SUCTION_PCM_ID);
            rightSuctionSolenoid = new Solenoid(Constants.RIGHT_SUCTION_PCM_ID);
		} catch (Exception e) {
			System.err.println("An error occurred in the Intake constructor");
		}
	}

	public double getRightTriggerAxis() {
		return OI.getInstance().getOperatorGamepad().getRightTriggerAxis() > 0.3
				? OI.getInstance().getOperatorGamepad().getRightTriggerAxis()
				: 0;
	}

	public double getLeftTriggerAxis() {
		return OI.getInstance().getOperatorGamepad().getLeftTriggerAxis() > 0.3
				? OI.getInstance().getOperatorGamepad().getLeftTriggerAxis()
				: 0;
	}

	public void setSpeed() {
		topIntake.set(ControlMode.PercentOutput, -0.75 * (getRightTriggerAxis() - getLeftTriggerAxis()));
		bottomIntake.set(ControlMode.PercentOutput, 0.75 * (getRightTriggerAxis() - getLeftTriggerAxis()));
	}

	public void setSpeed(double speed) {
		topIntake.set(ControlMode.PercentOutput, speed);
		bottomIntake.set(ControlMode.PercentOutput, -speed);
	}

	@Override
	protected void initDefaultCommand() {

	}

    public void updateStatus(Robot.OperationMode operationMode) {
		if (operationMode == Robot.OperationMode.COMPETITION) {
			SmartDashboard.putNumber("Current Left Suction Current: ", leftSuction.getOutputCurrent());
			SmartDashboard.putNumber("Current Right Suction Current: ", rightSuction.getOutputCurrent());
			SmartDashboard.putNumber("Current Top Roller Current: ", topIntake.getOutputCurrent());
		}
	}


	public boolean isSucc() {
		return (leftSuction.getOutputCurrent() > 5 || rightSuction.getOutputCurrent() > 5) == true;
	}

	public boolean hasCargo() {
		return topIntake.getOutputCurrent() > 5;
	}

	public void setSuction(IntakeState state) {
		this.succState = state;
		if (succState == IntakeState.SUCC_IN) {
			leftSuctionSolenoid.set(false);
            rightSuctionSolenoid.set(false);
            leftSuction.set(ControlMode.PercentOutput, 1);
            rightSuction.set(ControlMode.PercentOutput, 1);
		} else if (succState == IntakeState.SUCC_OUT) {
			leftSuctionSolenoid.set(true);
            rightSuctionSolenoid.set(true);
            leftSuction.set(ControlMode.PercentOutput, 0);
            rightSuction.set(ControlMode.PercentOutput, 0);
        }
	}

	@Override
	public void controlLoopUpdate() {
		// if(getSuccState() == IntakeState.SUCC_IN){
		// 	if(leftSuction.getOutputCurrent()>=0.375 && rightSuction.getOutputCurrent()>0.24){
		// 		Robot.oi.getDriverGamepad().setRumble(RumbleType.kLeftRumble, 1);
		// 		Robot.oi.getDriverGamepad().setRumble(RumbleType.kRightRumble, 1);
		// 	}else if(leftSuction.getOutputCurrent()>=0.38){
		// 		Robot.oi.getDriverGamepad().setRumble(RumbleType.kLeftRumble, 0.5);				
		// 	}else if(rightSuction.getOutputCurrent()>=0.24){
		// 		Robot.oi.getDriverGamepad().setRumble(RumbleType.kRightRumble, 0.25);
		// 	}else{
		// 		Robot.oi.getDriverGamepad().setRumble(RumbleType.kLeftRumble, 0);
		// 		Robot.oi.getDriverGamepad().setRumble(RumbleType.kRightRumble, 0);				
		// 	}
		// }else{
		// 	Robot.oi.getDriverGamepad().setRumble(RumbleType.kLeftRumble, 0);
		// 	Robot.oi.getDriverGamepad().setRumble(RumbleType.kRightRumble, 0);
		// }
	}

	@Override
	public void setPeriodMs(long periodMs) {
		// TODO Auto-generated method stub

	}

	public void setSpeed(double mSpeed, double mSpeed2) {
		topIntake.set(ControlMode.PercentOutput, mSpeed);
		bottomIntake.set(ControlMode.PercentOutput, -mSpeed2);

	}

	public void setTopSpeed(double speed) {
		topIntake.set(ControlMode.PercentOutput, speed);
	}

	public void setBottomSpeed(double speed) {
		bottomIntake.set(ControlMode.PercentOutput, -speed);
	}

	public IntakeState getSuccState() {
		return succState;
	}
}