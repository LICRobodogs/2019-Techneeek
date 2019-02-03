package frc.Autonomous.Paths;

import java.util.ArrayList;

// import org.usfirst.frc.team195.robot.Autonomous.Paths.PathAdapter;
import frc.util.TrajectoryFollowingMotion.Path;
import frc.util.TrajectoryFollowingMotion.PathBuilder;
import frc.util.TrajectoryFollowingMotion.PathBuilder.Waypoint;
import frc.util.TrajectoryFollowingMotion.PathContainer;
import frc.util.math.RigidTransform2d;
import frc.util.math.Rotation2d;
import frc.util.math.Translation2d;

public class StraightPath implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(20,276,0,0));
        sWaypoints.add(new Waypoint(122,279,0,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(20, 276), Rotation2d.fromDegrees(0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
}