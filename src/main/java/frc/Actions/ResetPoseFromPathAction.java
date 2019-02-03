package frc.Actions;

import edu.wpi.first.wpilibj.Timer;
import frc.Actions.Framework.RunOnceAction;
import frc.robot.subsystems.DriveBaseSubsystem;
import frc.util.TrajectoryFollowingMotion.PathContainer;
import frc.util.TrajectoryFollowingMotion.PathFollowerRobotState;
import frc.util.math.RigidTransform2d;

/**
 * Resets the robot's current pose based on the starting pose stored in the pathContainer object.
 *
 */
public class ResetPoseFromPathAction extends RunOnceAction {

    protected PathContainer mPathContainer;

    public ResetPoseFromPathAction(PathContainer pathContainer) {
        mPathContainer = pathContainer;
    }

    @Override
    public synchronized void runOnce() {
        RigidTransform2d startPose = mPathContainer.getStartPose();
        PathFollowerRobotState.getInstance().reset(Timer.getFPGATimestamp(), startPose);
        DriveBaseSubsystem.getInstance().setGyroAngle(startPose.getRotation());
    }
}
