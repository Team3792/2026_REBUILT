// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ShooterSubsystem.Hood;

// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** Add your docs here. */
public class HoodConstants {

    public static final int  kHoodCANID = 0;
    public static final int kEncoderCANID = 0;


    //Hood Angle Constants for Zones
    public static final double kZone1Angle = 0;

    public static SparkMaxConfig configHood() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(30);
        config.idleMode(IdleMode.kBrake);
        config.inverted(false);
        return config;
    }

    public static final double frictionFF = 0;
}
