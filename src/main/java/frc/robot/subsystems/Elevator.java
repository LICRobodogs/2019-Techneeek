package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.elevator.JoystickElevator;
import frc.util.Constants;
import frc.util.drivers.DunkTalonSRX;
import frc.util.drivers.DunkVictorSPX;
import frc.util.drivers.IPositionControlledSubsystem;
import frc.util.drivers.LeaderDunkTalonSRX;
import frc.util.drivers.MotionParameters;
import frc.util.drivers.SRXGains;

/**
 *
 */
public class Elevator extends Subsystem implements IPositionControlledSubsystem {
    private static Elevator instance = null;

	public static Elevator getInstance() {
		if (instance == null)
			instance = new Elevator();
		return instance;
	}

    private boolean isHoldingPosition = false;
    private boolean atScale = false;

    private int homePosition = 3000;
	private int collectPosition = 1000;
	private int switchPosition = 16000;
	private int autoSwitchPostion = 20000;
	private int topOfFirstStagePosition = 28000;
	private int minimumDunkHeight = 26500;
	private int dunkPosition = 33500;
	private int climbPosition = 48000;
	private int maxUpTravelPosition = 47000;

	private int scaleMiddlePosition = 38500;
	private int scaleBottomPosition = 35000;
	private int scaleTopPosition = 51000;

	public final static int ELEVATOR_UP = 0;
	public final static int ELEVATOR_DOWN = 1;

	public int upPositionLimit = maxUpTravelPosition;
	public int downPositionLimit = homePosition;
	private int targetPosition = 0;
	private double arbitraryFeedForward = 0.09;

	private final static int onTargetThreshold = 500; // changed to 500 from 100 for testing on practice field
    
    //                                            slot          p      i     d      f    izone
	private final SRXGains upGains = new SRXGains(ELEVATOR_UP, 0.25, 0.0, 0.0, 0.0, 0);
	private final SRXGains downGains = new SRXGains(ELEVATOR_DOWN, 0.25, 0.0, 0.0, 0.0, 0);
	
	//Uses PID values to go to a position                              accel velo  gains
	private MotionParameters upMotionParameters = new MotionParameters(6000, 6000, upGains);
	private MotionParameters downMotionParameters = new MotionParameters(3000, 2000, downGains);	
	
	public final DunkVictorSPX elevatorVictorFollower = new DunkVictorSPX(Constants.ELEVATOR_VICTOR1_ID);
	public final DunkTalonSRX elevatorTalonFollower = new DunkTalonSRX(Constants.ELEVATOR_TALON2_ID);
	public final LeaderDunkTalonSRX elevatorLead = new LeaderDunkTalonSRX(Constants.ELEVATOR_TALON1_ID, elevatorTalonFollower, elevatorVictorFollower);

	public Elevator() {
		this.elevatorLead.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

		this.elevatorLead.configForwardSoftLimitEnable(true);
		this.elevatorLead.configForwardSoftLimitThreshold(upPositionLimit);

		this.elevatorLead.configReverseSoftLimitEnable(true);
		this.elevatorLead.configReverseSoftLimitThreshold(downPositionLimit);

		this.elevatorLead.setInverted(false);
		this.elevatorLead.setSensorPhase(false);

		this.elevatorTalonFollower.setInverted(true);

		this.elevatorLead.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10);
		this.elevatorLead.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10);

		this.elevatorLead.configMotionParameters(upMotionParameters);
		this.elevatorLead.configMotionParameters(downMotionParameters);

		this.elevatorLead.setNeutralMode(NeutralMode.Brake);
		this.elevatorLead.configClosedloopRamp(0.25);
		this.elevatorLead.configOpenloopRamp(0.5);

		this.elevatorLead.configVoltageCompSaturation(11.5);
		this.elevatorLead.enableVoltageCompensation(true);

		// this.elevatorLead.configPeakOutputReverse(-0.2);
		// this.elevatorLead.configPeakOutputForward(0.2);

		this.elevatorLead.setSelectedSensorPosition(0);
	}

	public void initDefaultCommand() {
		setDefaultCommand(new JoystickElevator());
	}

	//sets control mode to motion magic
	public void setElevator(ControlMode controlMode, double signal) {
		// if (controlMode == ControlMode.MotionMagic) {
		// 	this.manageMotion(signal);
		// }
		elevatorLead.set(controlMode, signal);
	}

	public void setElevator(ControlMode controlMode, double signal, DemandType demandType, double demand) {
		if (controlMode == ControlMode.MotionMagic) {
			this.manageMotion(signal);
		}
		elevatorLead.set(controlMode, signal, demandType, demand);
	}

	public void motionMagicControl() {
		this.manageMotion(targetPosition);
		this.elevatorLead.set(ControlMode.MotionMagic, targetPosition, DemandType.ArbitraryFeedForward, getArbitraryFeedForward());
	}

	public int getCurrentPosition() {
		return this.elevatorLead.getSelectedSensorPosition();
	}

	public double getCurrentDraw() {
		return this.elevatorLead.getOutputCurrent();
	}

	public boolean isHoldingPosition() {
		return this.isHoldingPosition;
	}

	public void setIsHoldingPosition(boolean isHoldingPosition) {
		this.isHoldingPosition = isHoldingPosition;
	}

	public int getTargetPosition() {
		return this.targetPosition;
	}

	public boolean setTargetPosition(int position) {
		if (!isValidPosition(position)) {
			return false;
		} else {
			this.targetPosition = position;
			return true;
		}
	}

	public void forceSetTargetPosition(int position) {
		this.targetPosition = position;
	}

	public void incrementTargetPosition(int increment) {
		int currentTargetPosition = this.targetPosition;
		int newTargetPosition = currentTargetPosition + increment;
		if (isValidPosition(newTargetPosition) && isWristSafe(newTargetPosition)) {
			this.targetPosition = newTargetPosition;
		}
	}

	public boolean isValidPosition(int position) {
		boolean withinBounds = position <= upPositionLimit && position >= downPositionLimit;
		return withinBounds;
	}

	public int getHomePosition() {
		return this.homePosition;
	}

	public int getCollectPosition() {
		return this.collectPosition;
	}

	public int getSwitchPosition() {
		return this.switchPosition;
	}

	public int getAutoSwitchPosition() {
		return this.autoSwitchPostion;
	}

	public int getTopOfFirstStagePosition() {
		return this.topOfFirstStagePosition;
	}

	public int getMinimumDunkPosition() {
		return this.minimumDunkHeight;
	}

	public int getHatchPosition(int level) {
		if(level==1){
			return Constants.HATCH_LEVEL1_SETPOINT;
		}else if(level==2){
			return Constants.HATCH_LEVEL2_SETPOINT;
		}else if(level==3){
			return Constants.HATCH_LEVEL3_SETPOINT;
		}
		return getCurrentPosition();
	}

	public int getCargoPosition(int level) {
		if(level==1){
			return Constants.CARGO_LEVEL1_SETPOINT;
		}else if(level==2){
			return Constants.CARGO_LEVEL2_SETPOINT;
		}else if(level==3){
			return Constants.CARGO_LEVEL3_SETPOINT;
		}
		return getCurrentPosition();
	}

	public int getClimbPosition() {
		return this.climbPosition;
	}

	public int getMaxUpTravelPosition() {
		return this.maxUpTravelPosition;
	}

	public int getScaleMiddlePosition() {
		return this.scaleMiddlePosition;
	}

	public int getScaleBottomPosition() {
		return this.scaleBottomPosition;
	}

	public int getScaleTopPosition() {
		return this.scaleTopPosition;
	}

	public double getArbitraryFeedForward() {
		return this.arbitraryFeedForward;
	}

	
	public void manageMotion(double targetPosition) {
		double currentPosition = getCurrentPosition();
		if (currentPosition < targetPosition) {
			elevatorLead.selectMotionParameters(upMotionParameters);
		} else {
            elevatorLead.selectMotionParameters(downMotionParameters);
		}
	}

	//Makes sure the wrist is safe, if it is not dont do anything
	public boolean isWristSafe(double targetElevatorPosition) {
		// boolean atRisk = Robot.arm.getCurrentPosition() < Robot.arm.getSafePosition();
		boolean atRisk = false;
		System.out.println("is wrist at risk: " + atRisk);
		if (atRisk && targetElevatorPosition < minimumDunkHeight && getCurrentPosition() > topOfFirstStagePosition) {
			return false;
		} else {
			return true;
		}
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Elevator Position", this.getCurrentPosition());
		SmartDashboard.putNumber("Elevator Velocity", this.getCurrentVelocity());
		SmartDashboard.putNumber("Elevator Current", this.getCurrentDraw());
		SmartDashboard.putNumber("Elevator Voltage", this.elevatorLead.getMotorOutputVoltage());
	}

	@Override
	public double getCurrentVelocity() {
		double currentVelocity = this.elevatorLead.getSelectedSensorVelocity();
		return currentVelocity;
	}

	@Override
	public boolean isInPosition(int targetPosition) {
		int currentPosition = this.getCurrentPosition();
		int positionError = Math.abs(currentPosition - targetPosition);
		if (positionError < onTargetThreshold) {
			return true;
		} else {
			return false;
		}
	}
}
