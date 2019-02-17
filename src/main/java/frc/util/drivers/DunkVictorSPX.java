package frc.util.drivers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;

public class DunkVictorSPX extends WPI_VictorSPX {

	private static final int DEFAULT_TIMEOUT_MS = 0;
	private  PIDController controller;
	private double rampBand;

	public DunkVictorSPX(int deviceNumber) {
		super(deviceNumber);
		this.configNominalOutputForward(0.0);
		this.configNominalOutputReverse(0.0);
		this.configMotionProfileTrajectoryPeriod(0);

	}
        /**
         * Constructor for a PID controlled motor, with a controllable multiplier.
         *
         * @param motor The motor being set.
         * @param rampBand The acceptable range for a motor change in one loop
         * @param controller The PIDController this was passed as output to
         */
	// public DunkVictorSPX(int deviceNumber, double rampBand, PIDController controller) {
	// 	this(deviceNumber);
	// 	this.rampBand = rampBand;
	// 	this.controller = controller;
	// 	controller.setOutputRange(0-rampBand, 0+rampBand);		
	// }
	// public void pidWrite(double pidInput) { 
	// 	this.set(pidInput);
	// 	controller.setOutputRange(pidInput - rampBand, pidInput + rampBand);
	// }

	public ErrorCode configPIDF(int slotIdx, double P, double I, double D, double F) {
		ErrorCode errorCode = ErrorCode.OK;

		errorCode = this.config_kP(slotIdx, P);
		if (errorCode != ErrorCode.OK) {
			return errorCode;
		}

		errorCode = this.config_kI(slotIdx, I);
		if (errorCode != ErrorCode.OK) {
			return errorCode;
		}

		errorCode = this.config_kD(slotIdx, D);
		if (errorCode != ErrorCode.OK) {
			return errorCode;
		}

		errorCode = this.config_kF(slotIdx, F);
		return errorCode;
	}

	public ErrorCode configNominalOutputForward(double percentOut) {
		return super.configNominalOutputForward(percentOut, DEFAULT_TIMEOUT_MS);
	}

	public ErrorCode configNominalOutputReverse(double percentOut) {
		return super.configNominalOutputReverse(percentOut, DEFAULT_TIMEOUT_MS);
	}

	public ErrorCode configPeakOutputForward(double percentOut) {
		return super.configPeakOutputForward(percentOut, DEFAULT_TIMEOUT_MS);
	}

	public ErrorCode configPeakOutputReverse(double percentOut) {
		return super.configPeakOutputReverse(percentOut, DEFAULT_TIMEOUT_MS);
	}

	public ErrorCode setGains(SRXGains gains) {
		return this.configPIDF(gains.parameterSlot, gains.P, gains.I, gains.D, gains.F);
	}

	public ErrorCode configSelectedFeedbackSensor(FeedbackDevice feedbackDevice, int pidIdx) {
		return super.configSelectedFeedbackSensor(feedbackDevice, pidIdx, DEFAULT_TIMEOUT_MS);
	}

	public ErrorCode configOpenloopRamp(double secondsFromNeutralToFull) {
		return super.configOpenloopRamp(secondsFromNeutralToFull, DEFAULT_TIMEOUT_MS);
	}

	public ErrorCode config_kP(int slotIdx, double value) {
		return super.config_kP(slotIdx, value, DEFAULT_TIMEOUT_MS);
	}

	public ErrorCode config_kD(int slotIdx, double value) {
		return super.config_kD(slotIdx, value, DEFAULT_TIMEOUT_MS);
	}

	public ErrorCode config_kF(int slotIdx, double value) {
		return super.config_kF(slotIdx, value, DEFAULT_TIMEOUT_MS);
	}

	public ErrorCode config_kI(int slotIdx, double value) {
		return super.config_kI(slotIdx, value, DEFAULT_TIMEOUT_MS);
	}

	public ErrorCode setSelectedSensorPosition(int sensorPos, int pidIdx) {
		return super.setSelectedSensorPosition(sensorPos, pidIdx, DEFAULT_TIMEOUT_MS);
	}

	public ErrorCode configMotionAcceleration(int sensorUnitsPer100msPerSec) {
		return super.configMotionAcceleration(sensorUnitsPer100msPerSec, DEFAULT_TIMEOUT_MS);
	}

	public ErrorCode configMotionCruiseVelocity(int sensorUnitsPer100ms) {
		return super.configMotionCruiseVelocity(sensorUnitsPer100ms, DEFAULT_TIMEOUT_MS);
	}

	public ErrorCode configMotionProfileTrajectoryPeriod(int baseTrajDurationMs) {
		return super.configMotionProfileTrajectoryPeriod(baseTrajDurationMs, DEFAULT_TIMEOUT_MS);
	}

	public double configGetParameter(ParamEnum param, int ordinal) {
		return super.configGetParameter(param, ordinal, DEFAULT_TIMEOUT_MS);
	}

}