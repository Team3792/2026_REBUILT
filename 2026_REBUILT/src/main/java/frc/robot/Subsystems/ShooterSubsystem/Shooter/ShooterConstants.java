// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.ShooterSubsystem.Shooter;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

/** Add your docs here. */
public class ShooterConstants {

    //TODO make accurate
    public static final double kLightSensorThreshold = 0.5;
    public static final double lightSensorDetectionRange = 0.01;//meters
    public static final double kballSeperationTime = 0.1;
    public static final double kFlywheelBallLoadCurrent = 0;//amps

    //pid & FF
    public static final double kV = 0.1163;
    public static final double kS = 0.23;
    public static final double kMMAccel = 200;
    public static final double kMMJerk = 2100;
    public static final double kP = 0.2; 
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFrictionFF = 0;

    public static TalonFXConfiguration applyRightMotorConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 70;

        Slot0Configs slot0 = config.Slot0;
        slot0.kS = kS;
        slot0.kV = kV;
        slot0.kP = kP;
        slot0.kI = kI;
        slot0.kD = kD;

        MotionMagicConfigs mmconfig = config.MotionMagic;
        mmconfig.MotionMagicAcceleration = kMMAccel;
        mmconfig.MotionMagicJerk = kMMJerk;
        return config;
    }
    public static TalonFXConfiguration applyLeftMotorConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 70;

        Slot0Configs slot0 = config.Slot0;
        slot0.kS = kS;
        slot0.kV = kV;
        slot0.kP = kP;
        slot0.kI = kI;
        slot0.kD = kD;

        MotionMagicConfigs mmconfig = config.MotionMagic;
        mmconfig.MotionMagicAcceleration = kMMAccel;
        mmconfig.MotionMagicJerk = kMMJerk;
        return config;
    }

    private static final double kFlywheelDiameterInches = 4; //diameter in inches
    public static final double kFlywheelDiameter =  Units.inchesToMeters(kFlywheelDiameterInches); //kFlywheelDiameterInches * 0.0254; //convert inches to meters
    public static final double kFlywheelRadius = kFlywheelDiameter / 2;

    public static final double kFallBackHubVoltage = 6.0;
    public static final double kRPSIncrement = 1;

    public static final double maxShooterRPS = 6000;
    //rpm value for flywheel while not shooting; less time to spin up to target rpm
    public static final double coastRPM = 0;

    public static final double kHubRPS = 45;
    public static final double kRPSTolerance = 2;

    //For calibrated shots, see separate ShotCalculator class

    //TODO: instead of 2, use best fit curve to return correct amount of iterations needed
    public static double targetHubPoseIterations(double robotVelocity){
        return robotVelocity * 2;
    }

    //TODO: make accurate
    public static final double ktimeVsDistance = 0; //how long it takes fuel to travel one meter
                                                               //meters
    public static  double fuelFlightTimeVsDistance(double robotToHubDistance){
        return robotToHubDistance * ktimeVsDistance;
    }
}
