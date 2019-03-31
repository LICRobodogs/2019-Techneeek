package frc.util;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;


//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// Use OutlineViewer program that shows the NetworkTables hierarchy and all the values associated with each key
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
public class ShuffleBoardUpdater {
    private NetworkTableInstance instance;
    private NetworkTable dashBoard; //values written to the SmartDashboard or Shuffleboard using the SmartDashboard.put() set of methods.
    private NetworkTable liveWindow; //Typically these are Subsystems and the associated sensors and actuators. -stores Test mode (Test on the Driver Station) values. 
    private NetworkTable limeLight; //
    
    //IMPORT ENTRIES
    NetworkTableEntry yaw,distance;

    public ShuffleBoardUpdater() {
        instance = NetworkTableInstance.getDefault();
        dashBoard = instance.getTable("SmartDashboard");
        liveWindow = instance.getTable("LiveWindow");
        limeLight = instance.getTable("limelight");

        yaw = dashBoard.getEntry("YAW");
        
        instance.startClientTeam(2579);
    }
    public void addDashBoardListeners() {
        dashBoard.addEntryListener("distance", (table, key, entry, value, flags) -> {
            System.out.println("Distance value changed to:" + value.getValue());
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        yaw.addListener(event -> {
            System.out.println("YAW changed value" + event.value.getValue());
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }


    public void update() {
        sendDriveTrainData();
        sendLimeLightData();
        sendArmData();
        sendElevatorData();
        sendIntakeData();
        sendMiscData();
    }
    public void sendDriveTrainData() {

    }
    public void sendLimeLightData() {

    }
    public void sendArmData() {

    }
    public void sendElevatorData() {
        SmartDashboard.putNumber("Elevator Position", Robot.elevator.getCurrentPosition());
		SmartDashboard.putNumber("Elevator Velocity", Robot.elevator.getCurrentVelocity());
		SmartDashboard.putNumber("Elevator Current", Robot.elevator.getCurrentDraw());
		SmartDashboard.putNumber("Elevator Voltage", Robot.elevator.elevatorLead.getMotorOutputVoltage());

    }
    public void sendIntakeData() {

    }
    public void sendMiscData() {

    }

}