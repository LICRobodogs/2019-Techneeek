package frc.Actions;

import frc.robot.subsystems.DriveBaseSubsystem;
import frc.util.math.Rotation2d;
import frc.Actions.Framework.Action;

/**
 * Drives the robot along the Path defined in the PathContainer object. The action finishes once the robot reaches the
 * end of the path.
 *
 */
public class TurnToHeadingAction implements Action {

	private DriveBaseSubsystem mDrive = DriveBaseSubsystem.getInstance();
    private double heading;
    boolean inErrorZone = false;
    int count = 0;
    private boolean isFinished = false;

	public TurnToHeadingAction(double rotationDeg) {
		heading = rotationDeg;
	}

	@Override
	public boolean isFinished() {
		// return mDrive.isDoneWithTurn();
		return isFinished;
	}

	@Override
	public void update() {
        double error = mDrive.turnController.getError();
        inErrorZone = Math.abs(error) < 2;
        if (inErrorZone) {
            count++;
            isFinished = count >= 5;
        } else {
            count = 0;
        }


		// Nothing done here, controller updates in mEnabedLooper in robot
	}

	@Override
	public void done() {

	}

	@Override
	public void start() {
		mDrive.rotateDegrees(Rotation2d.fromDegrees(heading).getDegrees());
	}
}
