package frc.Actions;

import frc.Actions.Framework.Action;
import frc.robot.subsystems.DriveBaseSubsystem;
import frc.util.TrajectoryFollowingMotion.Path;
import frc.util.TrajectoryFollowingMotion.PathContainer;
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
        System.out.println(p.getClass().getName());
    }

    @Override
    public boolean isFinished() {
        // return mDrive.isDoneWithPath();
        return true;
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
