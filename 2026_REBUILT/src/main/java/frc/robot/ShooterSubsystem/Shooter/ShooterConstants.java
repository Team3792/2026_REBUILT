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

}
