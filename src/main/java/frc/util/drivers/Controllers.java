package frc.util.drivers;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SPI;
import frc.controller.Ps4_Controller;
import frc.robot.subsystems.DriveBaseSubsystem;

public class Controllers {
    private static Controllers instance = null;
    private PIDController pid;

public static Controllers getInstance() {
        if (instance == null) 
            instance = new Controllers();
        
            return instance;
    }
    private Controllers() {
        double lKP = 0.4, lKI = 0.0, lKD = 0.0, lKF = 0.0;
        double rKP = 0.4, rKI = 0.0, rKD = 0.0, rKF = 0.0;
        leftDrive1 = new DunkTalonSRX(0,0.3,pid);
        // leftDrive1 = new DunkTalonSRX(0);
        leftDrive2 = new DunkTalonSRX(3);
        rightDrive1 = new DunkTalonSRX(1,0.3,pid);
        rightDrive2 = new DunkVictorSPX(2);
        ps_controller = new Ps4_Controller(0);
        // mxp = new DunkGyro(SPI.Port.kMXP);
        // mxp = new AHRS(SPI.Port.kMXP);
        //  gyro 
    }
    public void passPID(PIDController newPID) {
        pid = newPID;
    }
    private Ps4_Controller ps_controller;
    private DunkTalonSRX leftDrive1;
    private DunkTalonSRX leftDrive2;
    private DunkTalonSRX rightDrive1;
    private DunkVictorSPX rightDrive2;
    // private ADIS16470_IMU gyro = new ADIS16470_IMU(ADIS16470_IMU.Axis.kZ, SPI.Port.kMXP);
    private AHRS gyro = new AHRS(SPI.Port.kMXP);
    
    public DunkTalonSRX getLeftDrive1() {
        return leftDrive1;
    }
    public DunkTalonSRX getLeftDrive2() {
        return leftDrive2;
    }
    public DunkTalonSRX getRightDrive1() {
        return rightDrive1;
    }
    public DunkVictorSPX getRightDrive2() {
        return rightDrive2;
    }
    public AHRS getGyro() {
        return gyro;
    }
    public Ps4_Controller getPS_Controller() {
        return ps_controller;
    }
    // public ADIS16470_IMU getGyro() {
    //     return gyro;
    // }
}