// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ShooterSubsystem.Shooter;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;

/** Add your docs here. */
public class ShooterConstants {
    //pid
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFrictionFF = 0;

    public static TalonFXConfiguration applyRightMotorConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
         config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;//TO-DO; make accurate
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        return config;
    }
    public static TalonFXConfiguration applyLeftMotorConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
         config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;//TO-DO; make accurate
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        return config;
    }

    private static final double kFlywheelDiameterInches = 4; //diameter in inches
    public static final double kFlywheelDiameter = kFlywheelDiameterInches * 0.0254; //convert inches to meters
    public static final double kFlywheelRadius = kFlywheelDiameter / 2;

    public static final double maxShooterRPM = 6000;
    //rpm value for flywheel while not shooting; less time to spin up to target rpm
    public static final double coastRPM = 0;

    //distance vs rpm 2D array
        //TODO: set good rpm values for given distance range
    public static final double[][] distanceVSRPM = {{24,48,72,96,120,144,168,192,216,240,264},
                                                    {0,0,0,0,0,0,0,0,0,0,0}};
        //TODO: set good hood angle values for given distance range
    public static final double[][] distanceVsHoodAngle = {{24,48,72,96,120,144,168,192,216,240,264},
                                                          {0,0,0,0,0,0,0,0,0,0,0}};

    //TODO: make accurate
    public static final double ktimeVsDistance = 0;
                                                               //meters
    public static final double fuelFlightTimeVsDistance(double robotToHubDistance){
        return robotToHubDistance * ktimeVsDistance;
    }




}
