package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.command.Command;
import frc.controller.Ps4_Controller;
import frc.robot.Robot;
import frc.util.drivers.Controllers;
//Should drive straight using joy stick with gyro controlling the left and
public class JoyStickWithGyro extends Command {
    private static Ps4_Controller ps;
    private static final double kAngleSetpoint = 0.0;
	private static final double kP = 0.005; // propotional turning constant
    private static AHRS gyro;
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
        ps = Controllers.getInstance().getPS_Controller();
        gyro = Controllers.getInstance().getGyro();        
    }

    public void execute() {
        double turningValue = (kAngleSetpoint - gyro.getYaw()) * kP;
		// Invert the direction of the turn if we are going backwards
		turningValue = Math.copySign(turningValue, ps.xSpeed());
        Robot.driveBaseSubsystem.drive(ps.xSpeed(), turningValue);
    }

}
