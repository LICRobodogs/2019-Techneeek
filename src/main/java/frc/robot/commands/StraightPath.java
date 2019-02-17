package frc.robot.commands;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.PathFiles;
import frc.robot.Robot;

public class StraightPath extends Command {

	private String pathName;
	private String lFilePath;
	private String rFilePath;
	private Notifier m_follower_notifier;
	private boolean isFinished;

	@Override
	protected boolean isFinished() {
		return Robot.driveBaseSubsystem.isFollowingPath();
	}

	public StraightPath(String pathName) {
		this.pathName = pathName;
		requires(Robot.driveBaseSubsystem);
		initialize();
	}

	public void initialize() {
		Robot.driveBaseSubsystem.initializePath(PathFiles.straight_line);
	}

	public void execute() {
		Robot.driveBaseSubsystem.followPath();
	}

}
