package frc.util.drivers;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

/**
 * @author BigBa
 *
 */
public class LeaderDunkTalonSRX extends DunkTalonSRX {
	// a list of talons to follow the leader
	private final List<BaseMotorController> followerList;

	// the constructor
	/**
	 * @param leaderDeviceNumber
	 * @param followerDeviceNumbers
	 */
	public LeaderDunkTalonSRX(int leaderDeviceNumber, BaseMotorController... followers) {
		// the superconstructor
		super(leaderDeviceNumber);

		// create an empty list of followers
		followerList = new ArrayList<BaseMotorController>();

		// for each controller in the array,
		// tell it to follow the leader, and add it to the list
		// of followers
		for (BaseMotorController follower : followers) {
			follower.follow(this);
			followerList.add(follower);
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * com.ctre.phoenix.motorcontrol.can.BaseMotorController#setNeutralMode(com.ctre
	 * .phoenix.motorcontrol.NeutralMode)
	 */
	@Override
	public void setNeutralMode(NeutralMode neutralMode) {
		super.setNeutralMode(neutralMode);
		for (BaseMotorController follower : followerList) {
			follower.setNeutralMode(neutralMode);
		}
	}

	@Override
	public void setInverted(boolean invert) {
		super.setInverted(invert);
		for (BaseMotorController follower : followerList) {
			follower.setInverted(invert);
		}
	}

	public ArrayList<Double> getOutputCurrents() {
		ArrayList<Double> outputCurrents = new ArrayList<Double>();
		outputCurrents.add(super.getOutputCurrent());
		for (BaseMotorController follower : followerList) {
			outputCurrents.add(((DunkTalonSRX) follower).getOutputCurrent());
		}
		return outputCurrents;
	}

}
