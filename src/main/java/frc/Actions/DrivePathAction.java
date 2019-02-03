package frc.Actions;

import frc.robot.subsystems.DriveBaseSubsystem;
import frc.util.TrajectoryFollowingMotion.PathContainer;
import frc.Actions.Framework.*;
import frc.util.TrajectoryFollowingMotion.*;
/**
 * Drives the robot along the Path defined in the PathContainer object. The action finishes once the robot reaches the
 * end of the path.
 *
 */
public class DrivePathAction implements Action {

    private PathContainer mPathContainer;
    private Path mPath;
    private DriveBaseSubsystem mDrive = DriveBaseSubsystem.getInstance();

    public DrivePathAction(PathContainer p) {
        mPathContainer = p;
        mPath = mPathContainer.buildPath();
        // ConsoleReporter.report(p.getClass().getName());
    }

    @Override
    public boolean isFinished() {
        return mDrive.isDoneWithPath();
    }

    @Override
    public void update() {
        // Nothing done here, controller updates in mEnabedLooper in robot
    }

    @Override
    public void done() {

    }

    @Override
    public void start() {
        mDrive.setWantDrivePath(mPath, mPathContainer.isReversed());
    }
}
