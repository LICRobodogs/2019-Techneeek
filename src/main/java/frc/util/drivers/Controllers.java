package frc.util.drivers;

import com.analog.adis16470.frc.ADIS16470_IMU;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

public class Controllers {
    private static Controllers instance = null;

    public static Controllers getInstance() {
        if (instance == null) 
            instance = new Controllers();
        
            return instance;
    }
    private Controllers() {
        leftDrive1 = new DunkTalonSRX(0);
        leftDrive2 = new DunkTalonSRX(3);
        rightDrive1 = new DunkTalonSRX(1);
        rightDrive2 = new DunkVictorSPX(2);
        // mxp = new DunkGyro(SPI.Port.kMXP);
        // mxp = new AHRS(SPI.Port.kMXP);
        //  gyro 
         

    }

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
    // public ADIS16470_IMU getGyro() {
    //     return gyro;
    // }
}