package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
// import frc.robot;
import frc.util.drivers.Controllers;

public class TurnToHeading extends Command {

	// private DriveBaseSubsystem mDrive = DriveBaseSubsystem.getInstance();
    private double heading;
    boolean inErrorZone = false;
    int count = 0;
    private boolean isFinished = false;

	public TurnToHeading(double heading) {
		this.heading = heading;
		requires(Robot.driveBaseSubsystem);

	}

	@Override
	public boolean isFinished() {
		// return mDrive.isDoneWithTurn();
		// System.out.println("DONE DUDE");
		return isFinished;
	}

	@Override
	public void execute() {
		
		// Robot.driveBaseSubsystem.rotateDegrees(heading);
		Robot.driveBaseSubsystem.setPercentSpeed(-.1,.1);
        // double error = Controllers.getInstance().getGyro().Error();
        // inErrorZone = Math.abs(error) < 2;
            isFinished = Math.abs(Controllers.getInstance().getGyro().getAngle()-heading) <= 5;


		// Nothing done here, controller updates in mEnabedLooper in robot
	}

	// @Override
	// public void done() {

	// }

	@Override
	public void start() {
		// mDrive.rotateDegrees(Rotation2d.fromDegrees(heading).getDegrees());
	}
}
