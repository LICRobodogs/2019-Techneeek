package frc.util;

import edu.wpi.first.wpilibj.Timer;
// import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
// import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import frc.util.TrajectoryFollowingMotion.MovingAverage;
/**
 * attempt to target your target looprate by estimating the amount of time it took from the last time it was called to the next time it was called
 * If loop only took 13 ms and target was 20, it'll sleep for 7. Tries to target looprate that was set
 */
public class ThreadRateControl {
	private double startTime;
	private double endTime;
	private int elapsedTimeMS;
	private boolean started;
	private double mPrevStartTime;
	private double mLoopTimeMS;
	private MovingAverage mAverageLoopTime;


	public ThreadRateControl() {
		startTime = 0;
		mPrevStartTime = 0;
		mLoopTimeMS = 0;
		endTime = 0;
		elapsedTimeMS = 0;
		started = false;
		mAverageLoopTime = new MovingAverage(20);
	}

	public synchronized void start(boolean resetStart) {
		if (resetStart)
			started = false;
		start();
	}

	public synchronized void start() {
		if (!started) {
			startTime = Timer.getFPGATimestamp();
			mPrevStartTime = startTime;
			started = true;
		} else {
			// ConsoleReporter.report("Thread rate control start called too many times!", MessageLevel.ERROR);
			System.err.println("ERROR: Thread rate control start called too many times!");
			//todo 
		}
	}

	public synchronized void doRateControl(int minLoopTime) {
		mLoopTimeMS = (startTime - mPrevStartTime) * 1000;
		mAverageLoopTime.addNumber(mLoopTimeMS);
		if (startTime != 0) {
			do {
				endTime = Timer.getFPGATimestamp();
				elapsedTimeMS = (int) ((endTime - startTime) * 1000);
				if (elapsedTimeMS < minLoopTime) {
					try {
						Thread.sleep(minLoopTime - elapsedTimeMS);
					} catch (Exception ex) {
						// ConsoleReporter.report(ex);
						System.err.println(ex);
					}
				}
			} while (elapsedTimeMS < minLoopTime);
		} else {
			// ConsoleReporter.report("Thread rate control called without setting a start time! Check your loops!", MessageLevel.ERROR);
			System.err.println("ERROR: Thread rate control called without setting a start time! Check your loops!");
		}
		mPrevStartTime = startTime;
		startTime = Timer.getFPGATimestamp();
	}

	public double getLoopTime() {
		return mLoopTimeMS;
	}

	public double getAverageLoopTime() {
		return mAverageLoopTime.getAverage();
	}
}
