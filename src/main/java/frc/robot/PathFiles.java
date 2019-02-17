package frc.robot;

import frc.util.Util;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

public class PathFiles{
    private static PathFiles instance = null;
    public static PathFiles getInstance() {
        if (instance == null)
             instance = new PathFiles();
        return instance;
   }
	
    public static paths straight_line;
    
    private PathFiles() { 
        straight_line = new paths("Straight_Line");
    }
    
    public class paths {
        public Trajectory left;
        public Trajectory right;
        paths(String name) {
            left = Pathfinder.readFromCSV(Util.lFile(name));
            right = Pathfinder.readFromCSV(Util.rFile(name));
        }
    }
}
