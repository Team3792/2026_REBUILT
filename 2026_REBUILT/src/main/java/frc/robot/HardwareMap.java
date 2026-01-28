// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.CancellationException;

import edu.wpi.first.wpilibj.CAN;

/** Add your docs here. */
public class HardwareMap {
    
    public static final int kDrivePort = 0;
    public static final int kOperatorPort = 1;

    public static final int kPDH = 1;

    //drive
    public static final CAN kPigeon = new CAN(1);

    //intake
    public static final CAN intakePivotID = new CAN(20);
    public static final CAN intakeDriveID = new CAN(21);//intake drive motor, and conveyor motor;
    //feeder

    public static final CAN conveyorID = new CAN(21);
    public static final CAN feederID = new CAN(22);
    //shooter
    
    public static final CAN shootLeaderID = new CAN(30);
    public static final CAN shootFollowerID = new CAN(31);
    public static final CAN hoodID = new CAN(32);
    public static final int hoodEncoderPort = 1;

    //climb
    public static final CAN climbLeft = new CAN(40);
    public static final CAN climbRight = new CAN(41);




}
