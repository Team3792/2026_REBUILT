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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.AimWhileMovingCommand;
import frc.robot.Localization.AntPoseEstimator;
import frc.robot.Localization.Vision;
import frc.robot.Managers.FieldManager;
import frc.robot.Managers.ZoneManager;
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
  //Vision vision = new Vision();
  //LED led = new LED();

  ZoneManager zoneManager = new ZoneManager();
  FieldManager fieldManager = new FieldManager();

  CommandPS5Controller driver = new CommandPS5Controller(0);
  CommandPS5Controller operator = new CommandPS5Controller(1);
  CommandXboxController aneesh = new CommandXboxController(2);

  //Trigger nearTrench = new Trigger(() -> zoneManager.isCloseToTrench());

  // Implement so that when the flywheel is up to speed the controller rumbles
  //Trigger shooterReady = new Trigger(() -> shooter.isAtTargetRPM(shooter.getTargetRPM(fieldManager.getRobotToHubDis(swerve.getPose()))));

  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureDriverBindings();
    configureOperatorBindings();
    configureAneeshBindings();

    //Register Named Commands


  //   boolean isCompetition = false;
  //   autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier((stream) -> isCompetition 
  //                 ? stream.filter(auto -> auto.getName().startsWith("comp")) : stream);

  //   SmartDashboard.putData("Auto Chooser", autoChooser);
    }

  //driver
  private void configureDriverBindings() {
    //Swerve
    swerve.setDefaultCommand(new DriveCommand(swerve, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));      
    driver.options().onTrue(Commands.runOnce(() -> swerve.resetHeading(0)));

    // driver.R2().onTrue(new AimWhileMovingCommand(swerve, FieldManager.getHubPose(), () -> -driver.getLeftY(), () -> -driver.getLeftX()));
    // driver.R2().onFalse(new DriveCommand(swerve, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    //Intake
    // driver.R2().onTrue(intake.runToPosition(IntakeConstants.kIntakePosition));
    // driver.R2().onFalse(intake.runToPosition(IntakeConstants.kStowedPosition));
    // driver.R2().whileTrue(new StartEndCommand(() -> intake.runToPosition(10), 
    //                       () -> intake.runToPosition(IntakeConstants.kStowedPosition)));

    driver.povUp().whileTrue(intake.movePivot(1));  
    driver.povDown().whileTrue(intake.movePivot(-1));
    driver.R1().whileTrue(intake.runIntake(IntakeConstants.kIntakeVoltage).alongWith(new StartEndCommand(() -> feeder.setConveyorVoltage(8), () -> feeder.setConveyorVoltage(0))));
    driver.L1().whileTrue(intake.runIntake(-IntakeConstants.kIntakeVoltage).alongWith(new StartEndCommand(() -> feeder.setConveyorVoltage(-8), () -> feeder.setConveyorVoltage(0))));
  }

  //operator
  private void configureOperatorBindings(){
    //Shoot
    //operator.R2().onTrue(shooter.setFlywheelRPM(100));//for testing

    //Feeder
    //operator.triangle().whileTrue(feeder.runFeeder(FeederConstants.kKickerVoltage, FeederConstants.kConveyorVoltage));

    //Manual Feeder
    operator.R2().whileTrue(new StartEndCommand(() -> feeder.setConveyorVoltage(8), () -> feeder.setConveyorVoltage(0)));
    operator.L2().whileTrue(new StartEndCommand(() -> feeder.setConveyorVoltage(-8), () -> feeder.setConveyorVoltage(0)));
    operator.R1().whileTrue(new StartEndCommand(() -> {feeder.setKickerVoltage(12); feeder.setConveyorVoltage(8);}, () -> {feeder.setKickerVoltage(0); feeder.setConveyorVoltage(0);}));
    operator.L1().whileTrue(new StartEndCommand(() -> feeder.setKickerVoltage(-12), () -> feeder.setKickerVoltage(0)));


    // //operator.square().whileTrue(new StartEndCommand(() -> shooter.setVoltage(4), () -> shooter.setVoltage(0)));
    // SmartDashboard.putNumber("Set RPS", 55);
    // operator.square().whileTrue(Commands.startEnd(() -> shooter.setRPS(SmartDashboard.getNumber("Set RPS", 0)), 
    //                             () -> shooter.setVoltage(0)).alongWith(Commands.startEnd(() -> hood.setAngle(SmartDashboard.getNumber("Set Hood Angle", 12)), 
    //                               () -> hood.setAngle(12))));

    // //Hood
    // operator.povUp().whileTrue(Commands.startEnd(() -> hood.setAngle(45), () -> hood.setAngle(12)));

    // SmartDashboard.putNumber("Set Hood Angle", 43);
    // operator.povRight().whileTrue(Commands.startEnd(() -> hood.setAngle(SmartDashboard.getNumber("Set Hood Angle", 12)), 
    //                               () -> hood.setAngle(12)));

    operator.circle().whileTrue(Commands.runOnce(() -> hood.setAngle(30), hood));
    operator.cross().onTrue(Commands.runOnce(() -> hood.setAngle(15), hood));

    

    //Climb
    // operator.povUp().whileTrue(climb.climbUpCommand());
    // operator.povDown().whileTrue(climb.climbDownCommand());
  }

  private void configureAneeshBindings(){
    //nearTrench.whileTrue(rumble(0.7, 5));

    aneesh.rightTrigger().whileTrue(rumble(0.75));

    aneesh.rightBumper().whileTrue(new StartEndCommand(() -> {feeder.setKickerVoltage(12); feeder.setConveyorVoltage(8);}, () -> {feeder.setKickerVoltage(0); feeder.setConveyorVoltage(0);}));
    aneesh.x().whileTrue(Commands.startEnd(() -> shooter.setRPS(SmartDashboard.getNumber("Set RPS", 0)), 
                                () -> shooter.setVoltage(0)).alongWith(Commands.startEnd(() -> hood.setAngle(SmartDashboard.getNumber("Set Hood Angle", 12)), 
                                  () -> hood.setAngle(12))));
    aneesh.leftBumper().whileTrue(new StartEndCommand(() -> {feeder.setKickerVoltage(-12); feeder.setConveyorVoltage(-8);}, () -> {feeder.setKickerVoltage(0); feeder.setConveyorVoltage(0);}));
  }


  // intensity on a scale of 0-1 and duration in seconds
  private Command rumble(double intensity, double duration){
      return Commands.startEnd(() -> aneesh.setRumble(RumbleType.kBothRumble, intensity), () -> aneesh.setRumble(RumbleType.kBothRumble, 0)).withTimeout(duration);
  }

  private Command rumble(double intensity){
    return Commands.startEnd(() -> aneesh.setRumble(RumbleType.kBothRumble, intensity), () -> aneesh.setRumble(RumbleType.kBothRumble, 0));
  }

  
  public Command getAutonomousCommand() {
    //return autoChooser.getSelected();
    return Commands.print("No autonomous command configured");
  }
}
