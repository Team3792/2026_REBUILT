// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Localization.Vision;
import frc.robot.Managers.FieldManager;
import frc.robot.Managers.ZoneManager;
import frc.robot.Subsystems.ClimbSubsytem.Climb;
import frc.robot.Subsystems.FeederSubsystem.Feeder;
import frc.robot.Subsystems.FeederSubsystem.FeederConstants;
import frc.robot.Subsystems.IntakeSubsystem.Intake;
import frc.robot.Subsystems.IntakeSubsystem.IntakeConstants;
import frc.robot.Subsystems.ShooterSubsystem.Hood.*;
import frc.robot.Subsystems.ShooterSubsystem.Shooter.*;
import frc.robot.Subsystems.SwerveSubsystem.DriveCommand;
import frc.robot.Subsystems.SwerveSubsystem.Swerve;

public class RobotContainer {

  Feeder feeder = new Feeder();
  Intake intake = new Intake();
  Climb climb = new Climb();
  Swerve swerve = new Swerve();
  Shooter shooter = new Shooter();
  Hood hood = new Hood();
  Vision vision = new Vision();

  ZoneManager zoneManager = new ZoneManager();
  FieldManager fieldManager = new FieldManager();

  CommandXboxController ellis = new CommandXboxController(0);
  CommandXboxController aneesh = new CommandXboxController(1);

  //Trigger nearTrench = new Trigger(() -> zoneManager.isCloseToTrench());

  // Implement so that when the flywheel is up to speed the controller rumbles
  //Trigger shooterReady = new Trigger(() -> shooter.isAtTargetRPM(shooter.getTargetRPM(fieldManager.getRobotToHubDis(swerve.getPose()))));
  Trigger readyToShoot = new Trigger(() -> (shooter.isAtTargetRPS(ShooterConstants.kHubRPS) 
                    && hood.isAtTargetAngle(HoodConstants.kHubHoodAngle)));
  Trigger robotNotMoving = new Trigger(() -> swerve.robotNotMoving());
  Trigger intakeDeployed = new Trigger(() -> intake.intakeDeployed());

  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureAneeshBindings();
    configureEllisBindings();

    //Register Named Commands

  //   boolean isCompetition = false;
  //   autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier((stream) -> isCompetition 
  //                 ? stream.filter(auto -> auto.getName().startsWith("comp")) : stream);

  //   SmartDashboard.putData("Auto Chooser", autoChooser);
    }

  private void configureAneeshBindings(){
    //nearTrench.whileTrue(rumble(0.7, 5));

    aneesh.rightTrigger().whileTrue(rumble(0.75, aneesh));
    
    robotNotMoving.whileTrue(Commands.startEnd(() -> rumble(.7, aneesh), () -> rumble(0, aneesh)));
    readyToShoot.whileTrue(rumble(0.7, 4, aneesh));
    //readyToShoot.whileTrue(Commands.startEnd(() -> {feeder.setKickerVoltage(12); feeder.setConveyorVoltage(8);}, () -> {feeder.setKickerVoltage(0); feeder.setConveyorVoltage(0);}));

    //Feeder
    aneesh.rightBumper().whileTrue(new StartEndCommand(() -> {feeder.setKickerVoltage(12); feeder.setConveyorVoltage(8);}, () -> {feeder.setKickerVoltage(0); feeder.setConveyorVoltage(0);}));
    aneesh.leftBumper().whileTrue(new StartEndCommand(() -> {feeder.setKickerVoltage(-12); feeder.setConveyorVoltage(-8);}, () -> {feeder.setKickerVoltage(0); feeder.setConveyorVoltage(0);}));

    //Shooter and Hood
    //Hub Shot
    aneesh.a().whileTrue(Commands.startEnd(() -> shooter.setRPS(50), 
                                () -> shooter.setVoltage(0)).alongWith(Commands.startEnd(() -> hood.setAngle(15), 
                                  () -> hood.setAngle(12))));

    //Changeable Shot                  
    SmartDashboard.putNumber("Set RPS", 45);
    SmartDashboard.putNumber("Set Hood Angle", 20);            
    aneesh.x().whileTrue(Commands.startEnd(() -> shooter.setRPS(SmartDashboard.getNumber("Set RPS", 0)), 
                                () -> shooter.setVoltage(0)).alongWith(Commands.startEnd(() -> hood.setAngle(SmartDashboard.getNumber("Set Hood Angle", 12)), 
                                  () -> hood.setAngle(12))));
    aneesh.y().onTrue(intake.zeroIntake());                           
  }

  private void configureEllisBindings(){
    // Swerve
    swerve.setDefaultCommand(new DriveCommand(swerve, () -> -ellis.getLeftY(), () -> -ellis.getLeftX(), () -> -ellis.getRightX(), () -> ellis.getRightTriggerAxis() > 0.1));      
    ellis.leftStick().onTrue(Commands.runOnce(() -> swerve.resetHeading(0), swerve));
    ellis.leftTrigger().whileTrue(swerve.crossCommand());
   // robotNotMoving.whileTrue(Commands.startEnd(() -> rumble(.75), () -> rumble(0)));
    //readyToShoot.whileTrue(rumble(0.7, 4, ellis));

    // ellis.rightStick().onTrue(new AimWhileMovingCommand(swerve, FieldManager.getHubPose(), () -> -ellis.getLeftY(), () -> -ellis.getLeftX()));
    // ellis.rightStick().onFalse(new DriveCommand(swerve, () -> -ellis.getLeftY(), () -> -ellis.getLeftX(), () -> -ellis.getRightX()));


    // Intake 
    ellis.povUp().whileTrue(intake.movePivot(1));  
    ellis.povDown().whileTrue(intake.movePivot(-1));
    ellis.rightBumper().whileTrue(intake.runIntake(IntakeConstants.kIntakeVoltage).alongWith(Commands.startEnd(() -> feeder.setConveyorVoltage(FeederConstants.kConveyorVoltage), () -> feeder.setConveyorVoltage(0))));
    ellis.leftBumper().whileTrue(intake.runIntake(-IntakeConstants.kIntakeVoltage).alongWith(Commands.startEnd(() -> feeder.setConveyorVoltage(-FeederConstants.kConveyorVoltage), () -> feeder.setConveyorVoltage(0))));
    //ellis.rightTrigger().whileTrue(Commands.startEnd(() -> intake.setPosition(IntakeConstants.kIntakePosition), 
                        //() -> intake.setPosition(IntakeConstants.kStowedPosition)));
  }


  // intensity on a scale of 0-1 and duration in seconds
  private Command rumble(double intensity, double duration, CommandXboxController controller){
      return Commands.startEnd(() -> controller.setRumble(RumbleType.kBothRumble, intensity), () -> controller.setRumble(RumbleType.kBothRumble, 0)).withTimeout(duration);
  }

  private Command rumble(double intensity, CommandXboxController controller){
    return Commands.startEnd(() -> controller.setRumble(RumbleType.kBothRumble, intensity), () -> controller.setRumble(RumbleType.kBothRumble, 0));
  }

  public Command getAutonomousCommand() {
    //return autoChooser.getSelected();
    return Commands.print("No autonomous command configured");
  }
}
