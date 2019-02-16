package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

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
	private AHRS gyro;

	public TurnToHeading(double heading) {
		this.heading = heading;
		requires(Robot.driveBaseSubsystem);
		gyro = Controllers.getInstance().getGyro();
	}

	@Override
	public boolean isFinished() {
		return isFinished;
	}

	@Override
	public void execute() {
		
		// Robot.driveBaseSubsystem.rotateDegrees(heading);
        double error = heading - gyro.getYaw();
        // inErrorZone = Math.abs(error) < 2;
        isFinished = Math.abs(Controllers.getInstance().getGyro().getAngle()-heading) <= 5;
	}

	// @Override
	// public void done() {

	// }

	@Override
	public void start() {
		// mDrive.rotateDegrees(Rotation2d.fromDegrees(heading).getDegrees());
	}
}
