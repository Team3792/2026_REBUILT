// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.CancellationException;

import com.ctre.phoenix6.configs.CANdleConfigurator;

import edu.wpi.first.wpilibj.CAN;
import frc.robot.HardwareMap.CANAddress;

/** Add your docs here. */
public class HardwareMap {
    
    public static final int kDrivePort = 0;
    public static final int kOperatorPort = 1;

    public static final int kPDH = 1;

    //drive
    //public static final int kPigeon = 12;
    public static final CANAddress kPigeon = new CANAddress(12, "pigeon", 0);

    //intake
    //public static final int intakePivotID = 20;
    public static final CANAddress intakePivotID = new CANAddress(20, "intakePivot", 0);
    //public static final int intakeDriveID = 21;
    public static final CANAddress intakeDriveID = new CANAddress(21, "intakeDrive", 0);
    //feeder

    //public static final CAN conveyorID = new CAN(21);

    //public static final int conveyorID = 22;
    public static final CANAddress conveyorID = new CANAddress(22, "conveyor", 0);
    //public static final int kickerID = 23;
    public static final CANAddress kickerID = new CANAddress(23, "kicker", 0);
    //shooter
    
    //public static final int shootLeaderID = 30;
    public static final CANAddress shootLeaderID = new CANAddress(30, "shootLeader", 0);
    //public static final int shootFollowerID = 31;
    public static final CANAddress shootFollowerID = new CANAddress(31, "shootFollower", 0);
    public static final int hoodID = 32;
   
    public static final int hoodEncoderPort = 1;


    //climb
    public static final int climbID = 40;

    public record CANAddress(int id, String name, int index){}
}
