package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.util.drivers.Controllers;
//Should drive straight using joy stick with gyro controlling the left and right
public class JoyStickWithGyro extends Command {
    private static final double kAngleSetpoint = 0.0;
	private static final double kP = 0.005; // propotional turning constant

	// gyro calibration constant, may need to be adjusted;
	// gyro value of 360 is set to correspond to one full revolution
	// private static final double kVoltsPerDegreePerSecond = 0.0128;

	@Override
    protected boolean isFinished() {
        return false;
    }

    public JoyStickWithGyro(){
        requires(Robot.driveBaseSubsystem);
    }
    

    public void initialize(){        
    }

    public void execute() {

        double turningValue = (kAngleSetpoint - Controllers.getInstance().getGyro().getAngle()) * kP;
		// Invert the direction of the turn if we are going backwards
		turningValue = Math.copySign(turningValue, Robot.getPsController().xSpeed());

        Robot.driveBaseSubsystem.drive(Robot.getPsController().xSpeed(), turningValue);
    }

}
