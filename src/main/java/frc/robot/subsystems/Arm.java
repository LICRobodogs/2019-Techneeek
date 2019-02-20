package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.util.Constants;
import frc.util.drivers.DunkVictorSPX;
import frc.util.drivers.IPositionControlledSubsystem;
import frc.util.drivers.LeaderDunkTalonSRX;
import frc.util.drivers.MotionParameters;
import frc.util.drivers.SRXGains;

public class Arm extends Subsystem implements IPositionControlledSubsystem {
	private static Arm instance = null;

	public static Arm getInstance() {
		if (instance == null)
			instance = new Arm();
		return instance;
	}
    public static enum ArmPistonState {
        BRAKE, RELEASE, SHOOT, RELOAD
    }

	public static enum ArmControlMode {
		MANUAL, SENSORED, HOLD, TEST
	}

	public static enum ArmSide {
		FRONT, BACK, NEITHER, SAME
	}

	private ArmControlMode controlMode = ArmControlMode.HOLD;
	// private ArmSide armSide = ArmSide.BACK; //UNCOMMENT IF YOU WANT TO SIMULATE FULL MATCH
	private ArmSide armSide = ArmSide.FRONT; //FOR TESTING ONLY
	private ArmSide desiredArmSide = ArmSide.FRONT;
	private boolean hasMoved = true;
	public static DoubleSolenoid brakePiston, shootPiston;
	private LeaderDunkTalonSRX armTalon;
	private DunkVictorSPX armFollower;
	
	private int homePosition = 50;
	private int restPosition = 950;
	private int safePosition = 1000;
	private int drivingPosition = 950;
	private int maxUpTravelPosition = 1850;
	private int collectPosition = 350;
	private int frontHatchPosition = 100;
	private int backHatchPosition = 1780;
	private int frontCargoPosition = 560;
	private int backCargoPosition = 1780;
	public int upPositionLimit = maxUpTravelPosition;
	public int downPositionLimit = homePosition;

	public final static int WRIST_PROFILE_UP = 0;
	public final static int WRIST_PROFILE_DOWN = 1;

	private int targetPosition = homePosition;
	private final static int onTargetThreshold = 20;

	private SRXGains upGains = new SRXGains(WRIST_PROFILE_UP, Constants.mArmUpKp, Constants.mArmUpKi, Constants.mArmUpKd, Constants.mArmUpKf, Constants.mArmUpIZone);
	private SRXGains downGains = new SRXGains(WRIST_PROFILE_DOWN, Constants.mArmDownKp, Constants.mArmDownKi, Constants.mArmDownKd, Constants.mArmDownKf, Constants.mArmDownIZone);

	private MotionParameters upMotionParameters = new MotionParameters(1000, 1000, upGains);
	private MotionParameters downMotionParameters = new MotionParameters(1000, 1000, downGains);
	
	public double mAngle;


	private DigitalInput homeLimit;

	private Arm() {
		try {
			armTalon = new LeaderDunkTalonSRX(Constants.WRIST_TALON_ID);
			armFollower = new DunkVictorSPX(Constants.WRIST_VICTOR_ID);
			shootPiston = new DoubleSolenoid(Constants.SHOOT_IN_PCM_ID, Constants.SHOOT_OUT_PCM_ID);
			brakePiston = new DoubleSolenoid(Constants.BRAKE_DEPLOY_PCM_ID, Constants.BRAKE_RELEASE_PCM_ID);
			armFollower.follow(armTalon);
			armFollower.setInverted(true);
			
			this.armTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10);
			this.armTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10);

			this.armTalon.configForwardSoftLimitEnable(true);
			this.armTalon.configForwardSoftLimitThreshold(upPositionLimit);

			this.armTalon.configReverseSoftLimitEnable(true);
			this.armTalon.configReverseSoftLimitThreshold(downPositionLimit);

			armTalon.setNeutralMode(NeutralMode.Brake);
			armTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

			armTalon.configClosedloopRamp(Constants.mArmRampRate, 10);

			armTalon.configPeakOutputForward(Constants.ARM_MOTOR_VOLTAGE_PERCENT_LIMIT, 10);
			armTalon.configPeakOutputReverse(-Constants.ARM_MOTOR_VOLTAGE_PERCENT_LIMIT, 10);

			armTalon.configMotionParameters(upMotionParameters);
			armTalon.configMotionParameters(downMotionParameters);
		} catch (Exception e) {
			System.err.println("An error occurred in the Arm constructor");
		}
	}

	public void setArmPiston(ArmPistonState state) {
		if (state == ArmPistonState.SHOOT) {
			shootPiston.set(Value.kReverse);
		} else if (state == ArmPistonState.RELOAD) {
			shootPiston.set(Value.kForward);
		} else if (state == ArmPistonState.BRAKE) {
			brakePiston.set(Value.kForward);
		} else if (state == ArmPistonState.RELEASE) {
			brakePiston.set(Value.kReverse);
		}
	}

	public boolean isShot() {
		return shootPiston.get() == Value.kForward;
	}

	public boolean isBrakeEngaged() {
		return brakePiston.get() == Value.kForward;
    }

	public void setArmAngle() {
		if (!(controlMode == ArmControlMode.MANUAL))
			armTalon.set(ControlMode.Position, (mAngle != 0 ? mAngle : 0));
	}

	public void setArmAngle(ArmControlMode controlMode, double angle) {
		// setControlMode(controlMode);
		// // this.mAngle = angle-offset;
		// if(controlMode == ArmControlMode.MANUAL && Math.abs(angle) < Constants.ARM_HOLDING_PWM){
		// 	setControlMode(ArmControlMode.HOLD);
		// }
		// if (this.controlMode == ArmControlMode.MANUAL && Math.abs(angle) > Constants.ARM_HOLDING_PWM) {
		// 	armTalon.set(ControlMode.PercentOutput, angle, DemandType.ArbitraryFeedForward,getFeedForward());
		// } else if (this.controlMode == ArmControlMode.TEST) {

		// } else if (this.controlMode == ArmControlMode.HOLD) {
		// 	if(getCurrentPosition()<homePosition){
		// 		armTalon.set(ControlMode.Position, homePosition, DemandType.ArbitraryFeedForward,getFeedForward());
		// 	}else if(getCurrentPosition()>maxUpTravelPosition){
		// 		armTalon.set(ControlMode.Position, maxUpTravelPosition, DemandType.ArbitraryFeedForward,getFeedForward());
		// 	}else{
		// 		armTalon.set(ControlMode.Position, armTalon.getSelectedSensorPosition(0), DemandType.ArbitraryFeedForward,getFeedForward());
		// 	}
		// } else if (this.controlMode == ArmControlMode.SENSORED) {
		// 	armTalon.set(ControlMode.Position, (mAngle != 0 ? mAngle : 0));
		// }
	}

	public void controlLoopUpdate() {
		// moveWithJoystick();
		/*
		 * }else if (controlMode == ArmControlMode.SENSORED) { //moveWithFeedBack(); //}
		 */
	}

	public void moveWithFeedBack() {
		setArmAngle(ArmControlMode.SENSORED, (mAngle));
	}

	public void setSetpoint(double angle) {
		mAngle = (angle) * Constants.ARM_NATIVE_TO_ANGLE_FACTOR;
	}

	public void moveWithJoystick() {
		// setArmAngle(ArmControlMode.MANUAL,OI.getInstance().getOperatorGamepad().getRightYAxis()
		// * 4096);
		// if(!(getArmAngle() < 2 &&
		// OI.getInstance().getOperatorGamepad().getRightYAxis() < 0.2)) {
		// if(Intake.isIntakeIn())
		// Intake.setIntakePiston(IntakePistonState.OUT);
		if(Math.abs(Robot.oi.getOperatorGamepad().getRightYAxis())>0 && !isBrakeEngaged()){
			armTalon.set(ControlMode.PercentOutput, Robot.oi.getOperatorGamepad().getRightYAxis());
        }else if(!isBrakeEngaged()){
			// setArmPiston(ArmPistonState.BRAKE);
		}
	}

	@Override
	protected void initDefaultCommand() {
		// setDefaultCommand(new JoystickArm());
	}

	//sets control mode to motion magic
	public void wristMove(ControlMode controlMode, double targetPosition) {
		this.manageMotion(targetPosition);
		armTalon.set(controlMode, targetPosition);
	}

	public void motionMagicControl() {
		manageMotion(targetPosition);
		armTalon.set(ControlMode.MotionMagic, targetPosition, DemandType.ArbitraryFeedForward, getFeedForward());
	}

	/*
	 * controls the position of the collector between upPositionLimit and
	 * downPositionLimit based on the scalar input between -1 and 1.
	 */
	public void motionMagicPositionControl(double positionScalar) {
		double encoderPosition = 0;

		if (positionScalar > 0) {
			encoderPosition = positionScalar * upPositionLimit;
		} else {
			encoderPosition = -positionScalar * downPositionLimit;
		}
		armTalon.set(ControlMode.MotionMagic, encoderPosition);
	}

	/*
	 * choose which set of gains to use based on direction of travel.
	 */
	public void manageMotion(double targetPosition) {
		double currentPosition = getCurrentPosition();

		manageLimits();
		if (currentPosition > targetPosition) {
			armTalon.selectMotionParameters(downMotionParameters);
		} else {
			armTalon.selectMotionParameters(upMotionParameters);
		}
	
		this.armTalon.configForwardSoftLimitThreshold(upPositionLimit);
		this.armTalon.configReverseSoftLimitThreshold(downPositionLimit);
	}

	//Prevents wrist from moving behind the home position whilst elevator is not above first stage
	
	private void manageLimits() {
		if (Robot.elevator.getCurrentPosition() >= Robot.elevator.getTopOfFirstStagePosition()) {
			this.upPositionLimit = maxUpTravelPosition;
		} else {
			this.upPositionLimit = restPosition;
			if (this.targetPosition < homePosition) {
				this.targetPosition = homePosition;
			}
		}
	}

	public int getTargetPosition() {
		return this.targetPosition;
	}

	//if valid position is inverted return false else, return true
	public boolean setTargetPosition(int position) {
		manageLimits();
		if (!isValidPosition(position)) {
			return false;
		} else {
			this.targetPosition = position;
			return true;
		}
	}

	public void updateStatus(Robot.OperationMode operationMode) {
		SmartDashboard.putNumber("Arm Angle: ", getArmAngle());
		SmartDashboard.putString("Side: ", getSide().toString());
		SmartDashboard.putString("Desired Side: ", getDesiredSide().toString());
		SmartDashboard.putNumber("Arm RAW Angle: ", getCurrentPosition());
		// SmartDashboard.putBoolean("onTarget", isOnTarget());
		SmartDashboard.putNumber("Arm Motor Current", armTalon.getOutputCurrent());
		SmartDashboard.putNumber("PWM:", armTalon.getMotorOutputVoltage());
		SmartDashboard.putNumber("FeedForward:", getFeedForward());
		// SmartDashboard.putBoolean("isShot", isShot());
		SmartDashboard.putBoolean("isBrakeEngaged", isBrakeEngaged());
		SmartDashboard.putString("TALON MODE: ", armTalon.getControlMode().toString());
		SmartDashboard.putString("ARM CONTROL MODE: ", controlMode.toString());
		// SmartDashboard.putNumber("mAngle: ", mAngle);
		if(this.controlMode == ArmControlMode.MANUAL){
			SmartDashboard.putNumber("Right Y axis", Robot.oi.getOperatorGamepad().getRightYAxis());
		}else if(this.controlMode == ArmControlMode.SENSORED){
			SmartDashboard.putNumber("isOnTarget result", Math.abs(getArmAngle() - Math.abs(getAngleSetpoint())));
			SmartDashboard.putNumber("Arm Setpoint", getAngleSetpoint());
		}
		if (operationMode == Robot.OperationMode.TEST) {
		}
	}

	public boolean isOnTarget() {
		if ((getAngleSetpoint() == 0 && isHome())
				|| (armTalon.getControlMode() == ControlMode.PercentOutput && controlMode == ArmControlMode.MANUAL))
			return true;
		else
			return (armTalon.getControlMode() == ControlMode.Position
					&& Math.abs(getAngleSetpoint() - Math.abs(getArmAngle())) < Constants.mArmOnTargetTolerance);
	}

	public double getAngleSetpoint() {
		return armTalon.getClosedLoopTarget(0) / Constants.ARM_NATIVE_TO_ANGLE_FACTOR;
	}

	private double getArmAngle() {
		int rawAngle = armTalon.getSelectedSensorPosition(0);
		// return
		// ((double)armTalon.getSelectedSensorPosition(0))/NATIVE_TO_ANGLE_FACTOR;
		return rawAngle / Constants.ARM_NATIVE_TO_ANGLE_FACTOR;
		// return angle < 0.1 ? 0:angle;
	}

	@Override
	public int getCurrentPosition() {
		int rawAngle = armTalon.getSelectedSensorPosition(0);
		// return
		// ((double)armTalon.getSelectedSensorPosition(0))/NATIVE_TO_ANGLE_FACTOR;
		return rawAngle;
		// return angle < 0.1 ? 0:angle;
	}

	private double getFeedForward() {
		double theta = Math.toRadians(getArmAngle());

		double gravityCompensation = Math.cos(theta);

		SmartDashboard.putNumber("Gravity Comp", gravityCompensation);

		return gravityCompensation * Constants.ARM_HOLDING_PWM;
	}

	public boolean isValidPosition(double signal, boolean manual){
		if(signal>0 && (getCurrentPosition()>=maxUpTravelPosition)){
			return false;
		}
		if(signal<0 && (getCurrentPosition()<=homePosition)){
			return false;
		}
		if(signal>0 && (getCurrentPosition()>900 && Robot.elevator.getCurrentPosition()<13000)){
			return false;
		}
		return true;
	}

	public boolean isValidPosition(int position) {
		return (position <= upPositionLimit && position >= downPositionLimit);
		// return true;
	}

	public void resetArmEncoder() {
		armTalon.setSelectedSensorPosition(0, 0, 10);
	}

	public void setStartConfigAngle() {
		armTalon.setSelectedSensorPosition(Constants.START_CONFIG_ANGLE);
	}

	public void setControlMode(ArmControlMode mode) {
		this.controlMode = mode;
		// resetArmEncoder();
	}

	//ASSUMING SIDE TOGGLE CAN ONLY BE PRESSED ONCE
	public void setDesiredArmSide(ArmSide side) {
		this.desiredArmSide = side;
	}

	public void setHasMoved(boolean hasMoved) {
		this.hasMoved = hasMoved;
	}

	public LeaderDunkTalonSRX getMasterTalon(){
		return armTalon;
	}

	public ArmSide getSide() {
		return armSide;
	}

	public ArmSide getDesiredSide() {
		return desiredArmSide;
	}

	public ArmControlMode getMode() {
		return controlMode;
	}

	public boolean isHome() {
		return homeLimit.get();
	}

	public int getFrontCargoPosition(){
		return frontCargoPosition;
	}

	public int getBackCargoPosition(){
		return backCargoPosition;
	}

	public int getFrontHatchPosition(){
		return frontHatchPosition;
	}

	public int getBackHatchPosition(){
		return backHatchPosition;
	}

	public int getFrontRestPosition(){
		return restPosition;
	}

	public int getCollectPosition(){
		return collectPosition;
	}

	public int getSafePosition(){
		return safePosition;
	}

	public int getDrivingPosition(){
		return drivingPosition;
	}

	public int getHomePosition(){
		return homePosition;
	}

	public int getTargetThreshold(){
		return onTargetThreshold;
	}

	@Override
	public void periodic() {
		if(getCurrentPosition() < 1100){
			armSide = ArmSide.FRONT;
		}else{
			armSide = ArmSide.BACK;
		}
	}

	@Override
	public double getCurrentVelocity() {
		return armTalon.getSelectedSensorVelocity();
	}

	@Override
	public boolean isInPosition(int targetPosition) {
		int currentPosition = getCurrentPosition();
		int positionError = Math.abs(currentPosition - targetPosition);
		return positionError < onTargetThreshold;
	}
}
