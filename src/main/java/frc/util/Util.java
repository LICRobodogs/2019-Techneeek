package frc.util;

import java.util.List;

import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;

/**
 * Contains basic functions that are used often.
 */
public class Util {
    /** Prevent this class from being instantiated. */
    private Util() {
    }

    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }
    public static File lFile(String pathToFollow) {
		// Util.startFileSearch(pathToFollow);
		String lFilePath = "/home/lvuser/deploy/output/"+pathToFollow+".left.pf1.csv";
		return new File(lFilePath);
	}
	public static File rFile(String pathToFollow) {
		String rFilePath = "/home/lvuser/deploy/output/"+pathToFollow+".right.pf1.csv";
		return new File(rFilePath);
	}
    public static void startFileSearch(String name) {	
        String directory = Filesystem.getDeployDirectory().toString();
        findFile(name,new File(directory));
}
public static void findFile(String name,File file) {
        File[] list = file.listFiles();
        if(list!=null)
        for (File fil : list)
        {
                if (fil.isDirectory())
                {
                        findFile(name,fil);
                }
                else if (name.equalsIgnoreCase(fil.getName()))
                {
                        System.out.println(fil.getParentFile());
                }
        }
}

    public static String joinStrings(String delim, List<?> strings) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < strings.size(); ++i) {
            sb.append(strings.get(i).toString());
            if (i < strings.size() - 1) {
                sb.append(delim);
            }
        }
        return sb.toString();
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean allCloseTo(List<Double> list, double value, double epsilon) {
        boolean result = true;
        for (Double value_in : list) {
            result &= epsilonEquals(value_in, value, epsilon);
        }
        return result;
    }
    public static int convertRPMToNativeUnits(double rpm) {
        return (int)(rpm * Constants.kSensorUnitsPerRotation / Constants.k100msPerMinute);
    }

    public static int convertNativeUnitsToRPM(double nativeUnits) {
        return (int)(nativeUnits / Constants.kSensorUnitsPerRotation * Constants.k100msPerMinute);
    }
}
