package frc.Autonomous.Modes;

import frc.Actions.DrivePathAction;
import frc.Actions.ResetPoseFromPathAction;
import frc.Actions.Framework.WaitAction;
import frc.Autonomous.Framework.AutoModeBase;
import frc.Autonomous.Framework.AutoModeEndedException;
import frc.Autonomous.Paths.StraightPath;
import frc.util.TrajectoryFollowingMotion.PathContainer;

public class BasicMode extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {
        PathContainer pathContainer = new StraightPath();
        //First thing when calling path - goes into robot state estimator and resets estimated position to the start position of generated path
		runAction(new ResetPoseFromPathAction(pathContainer)); //zeros odomoeter .. basically

		runAction(new DrivePathAction(pathContainer)); // action that actually follows path

		runAction(new WaitAction(15));
	}
}
