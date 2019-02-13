package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.commands.JoystickArm;
import frc.util.Constants;
import frc.util.ControlLoopable;

public class Arm extends Subsystem implements ControlLoopable {
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

	private ArmControlMode controlMode = ArmControlMode.MANUAL;

	public static DoubleSolenoid brakePiston, shootPiston;
	private TalonSRX armTalon;
	private VictorSPX armFollower;

	
	private static double offset;
	
	public double mAngle;


	private DigitalInput homeLimit;

	private Arm() {
		try {
			// shootPiston = new DoubleSolenoid(Constants.SHOOT_IN_PCM_ID, Constants.SHOOT_OUT_PCM_ID);

			armTalon = new TalonSRX(Constants.WRIST_TALON_ID);
			armFollower = new VictorSPX(Constants.WRIST_VICTOR_ID);

			armFollower.follow(armTalon);
			armFollower.setInverted(true);
			armTalon.setNeutralMode(NeutralMode.Brake);
			armTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
			armTalon.config_kP(0, Constants.mArmKp, 10);
			armTalon.config_kI(0, Constants.mArmKi, 10);
			armTalon.config_kD(0, Constants.mArmKd, 10);
			armTalon.config_kF(0, Constants.mArmKf, 10);
			armTalon.config_IntegralZone(0, Constants.mArmIZone, 10);
			armTalon.configClosedloopRamp(Constants.mArmRampRate, 10);
			// armTalon.configNominalOutputForward(0, 10);
			// armTalon.configNominalOutputReverse(0, 10);
			armTalon.configPeakOutputForward(Constants.ARM_MOTOR_VOLTAGE_PERCENT_LIMIT, 10);
			armTalon.configPeakOutputReverse(-Constants.ARM_MOTOR_VOLTAGE_PERCENT_LIMIT, 10);
			// armTalon.set(ControlMode.PercentOutput, 0);

		} catch (Exception e) {
			System.err.println("An error occurred in the Arm constructor");
		}
	}

	public void setArmPiston(ArmPistonState state) {
		if (state == ArmPistonState.SHOOT) {
			shootPiston.set(Value.kForward);
		} else if (state == ArmPistonState.RELOAD) {
			shootPiston.set(Value.kReverse);
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
		return false;
		// return brakePiston.get() == Value.kReverse;
    }

	public void setArmAngle() {
		if (!(controlMode == ArmControlMode.MANUAL))
			armTalon.set(ControlMode.Position, (mAngle != 0 ? mAngle : 0));
	}

	public void setArmAngle(ArmControlMode controlMode, double angle) {
		setControlMode(controlMode);
		// this.mAngle = angle-offset;
		if(controlMode == ArmControlMode.MANUAL && Math.abs(angle) < Constants.ARM_HOLDING_PWM){
			setControlMode(ArmControlMode.HOLD);
			System.out.println("Should've changed control modes");
		}
		if (this.controlMode == ArmControlMode.MANUAL) {
			armTalon.set(ControlMode.PercentOutput, angle, DemandType.ArbitraryFeedForward,getFeedForward());
			System.out.println("In this for some reason");
		} else if (this.controlMode == ArmControlMode.TEST) {

		} else if (this.controlMode == ArmControlMode.HOLD) {
			armTalon.set(ControlMode.Position, armTalon.getSelectedSensorPosition(0), DemandType.ArbitraryFeedForward,getFeedForward());
		} else if (this.controlMode == ArmControlMode.SENSORED) {
			armTalon.set(ControlMode.Position, (mAngle != 0 ? mAngle : 0));
		}
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
		setDefaultCommand(new JoystickArm());
	}

	public void updateStatus(Robot.OperationMode operationMode) {
		SmartDashboard.putNumber("Arm Angle: ", getArmAngle());
		SmartDashboard.putNumber("Arm RAW Angle: ", armTalon.getSelectedSensorPosition());
		SmartDashboard.putNumber("Arm Setpoint", getAngleSetpoint());
		SmartDashboard.putNumber("isOnTarget result", Math.abs(getArmAngle() - Math.abs(getAngleSetpoint())));
		// SmartDashboard.putBoolean("onTarget", isOnTarget());
		SmartDashboard.putNumber("Arm Motor Current", armTalon.getOutputCurrent());
		SmartDashboard.putNumber("PWM:", armTalon.getMotorOutputVoltage());
		SmartDashboard.putNumber("FeedForward:", getFeedForward());
		// SmartDashboard.putBoolean("isHome", isHome());
		// SmartDashboard.putBoolean("isShot", isShot());
		SmartDashboard.putBoolean("isBrakeEngaged", isBrakeEngaged());
		SmartDashboard.putString("TALON MODE: ", armTalon.getControlMode().toString());
		SmartDashboard.putString("ARM CONTROL MODE: ", controlMode.toString());
		// SmartDashboard.putNumber("mAngle: ", mAngle);
		SmartDashboard.putNumber("Right Y axis", Robot.oi.getOperatorGamepad().getRightYAxis());
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

	private double getFeedForward() {
		double theta = Math.toRadians(getArmAngle());

		double gravityCompensation = Math.cos(theta);

		SmartDashboard.putNumber("Gravity Comp", gravityCompensation);

		return gravityCompensation * Constants.ARM_HOLDING_PWM;
	}

	public void resetArmEncoder() {
		offset = armTalon.getSensorCollection().getPulseWidthPosition() / Constants.ARM_NATIVE_TO_ANGLE_FACTOR;
		// mAngle=0;
		armTalon.setSelectedSensorPosition(0, 0, 10);
		// setSetpoint(0);
	}

	public void setStartConfigAngle() {
		armTalon.setSelectedSensorPosition(Constants.START_CONFIG_ANGLE);
	}

	public void setControlMode(ArmControlMode mode) {
		this.controlMode = mode;
		// resetArmEncoder();
	}

	public ArmControlMode getMode() {
		return controlMode;
	}

	public boolean isHome() {
		return homeLimit.get();
	}

	@Override
	public void setPeriodMs(long periodMs) {
		// TODO Auto-generated method stub

	}
}
