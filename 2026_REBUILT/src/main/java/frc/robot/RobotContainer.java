// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.regex.Pattern;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Commands.TurnToHubCommand;
import frc.robot.Localization.AntPoseEstimator;
import frc.robot.Localization.Vision;
import frc.robot.Subsystems.ClimbSubsytem.Climb;
import frc.robot.Subsystems.ClimbSubsytem.ClimbConstants;
import frc.robot.Subsystems.FeederSubsystem.Feeder;
import frc.robot.Subsystems.FeederSubsystem.FeederConstants;
import frc.robot.Subsystems.IntakeSubsystem.Intake;
import frc.robot.Subsystems.IntakeSubsystem.IntakeConstants;
import frc.robot.Subsystems.LEDSubsystem.LED;
import frc.robot.Subsystems.LEDSubsystem.LEDConstants;
import frc.robot.Subsystems.ShooterSubsystem.Hood.*;
import frc.robot.Subsystems.ShooterSubsystem.Shooter.*;
import frc.robot.Subsystems.SwerveSubsystem.DriveCommand;
import frc.robot.Subsystems.SwerveSubsystem.Swerve;
import frc.robot.Subsystems.SwerveSubsystem.SwerveConstants;

public class RobotContainer {

  Feeder feeder = new Feeder();
  Intake intake = new Intake();
  Climb climb = new Climb();
  Swerve swerve = new Swerve();
  Shooter shooter = new Shooter();
  Hood hood = new Hood();
  Vision vision = new Vision();
  LED led = new LED();

  CommandPS5Controller driver = new CommandPS5Controller(0);
  CommandPS5Controller operator = new CommandPS5Controller(1);

  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureDriverBindings();
    configureOperatorBindings();

    //Register Named Commands


    boolean isCompetition = false;
    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier((stream) -> isCompetition 
                  ? stream.filter(auto -> auto.getName().startsWith("comp")) : stream);

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  //driver
  private void configureDriverBindings() {
    // driver.R1().onTrue(intake.runToPosition(IntakeConstants.kIntakePosition));
    // driver.R1().onFalse(intake.runToPosition(IntakeConstants.kStowedPosition));

    // driver.R1().whileTrue(intake.runIntake(IntakeConstants.kIntakeVoltage));
    // driver.L1().whileTrue(intake.runIntake(-IntakeConstants.kIntakeVoltage));

    swerve.setDefaultCommand(new DriveCommand(swerve, () -> driver.getLeftY(), () -> driver.getLeftX(), () -> driver.getRightX()));
    driver.options().onTrue(Commands.runOnce(() -> swerve.resetHeading(0)));

    driver.R2().whileTrue(new TurnToHubCommand(swerve, new Pose2d(), () -> driver.getLeftY(), () -> driver.getLeftX()));

    // driver.L2().whileTrue(swerve.turnToTagCommand(vision.getTagYaw(Vision.frontCam)));
    // driver.R2().whileTrue(swerve.alignToTagCommand(new Pose2d()));

    //driver.triangle().onTrue(led.runPattern(LEDConstants.kClimbPattern));
    // driver.square().onTrue(led.runPattern(LEDConstants.kIntakePattern));
    // driver.cross().onTrue(led.runPattern(LEDConstants.kShootingPattern));
  }

  //operator
  private void configureOperatorBindings(){
    // operator.square().whileTrue(feeder.runFeeder(FeederConstants.kFeedVoltage));
    // operator.cross().whileTrue(feeder.runFeeder(-FeederConstants.kFeedVoltage)); 

    operator.povUp().whileTrue(climb.climbUpCommand());
    operator.povDown().whileTrue(climb.climbDownCommand());

    operator.L2().onTrue(hood.setPosition(HoodConstants.kZone1Angle));
    operator.R2().onTrue(shooter.setFlywheelRPM(100));//for testing
  }

  
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    //return Commands.print("No autonomous command configured");
  }
}
