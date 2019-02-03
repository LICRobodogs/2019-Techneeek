package frc.util;

import frc.util.loops.Looper;

public interface CustomSubsystem {
	void init();
	void subsystemHome();
	void registerEnabledLoops(Looper in);
}
