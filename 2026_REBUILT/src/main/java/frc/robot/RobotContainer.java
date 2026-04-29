// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.SpinUpCommand;
import frc.robot.Commands.SpinUpCommand.ShotType;
import frc.robot.Subsystems.ClimbSubsytem.Climb;
import frc.robot.Subsystems.FeederSubsystem.Feeder;
import frc.robot.Subsystems.FeederSubsystem.FeederConstants;
import frc.robot.Subsystems.IntakeSubsystem.Intake;
import frc.robot.Subsystems.IntakeSubsystem.IntakeConstants;
import frc.robot.Subsystems.ShooterSubsystem.ShotCalculator;
import frc.robot.Subsystems.ShooterSubsystem.Hood.*;
import frc.robot.Subsystems.ShooterSubsystem.Shooter.*;
import frc.robot.Subsystems.SwerveSubsystem.DriveCommand;
import frc.robot.Subsystems.SwerveSubsystem.Swerve;
import frc.robot.util.Rumbler;

public class RobotContainer {

  Feeder feeder = new Feeder();
  Intake intake = new Intake();
  Climb climb = new Climb();
  Swerve swerve = new Swerve();
  Shooter shooter = new Shooter();
  Hood hood = new Hood();

  CommandXboxController smellisEllis = new CommandXboxController(0);
  CommandXboxController neeshneesh = new CommandXboxController(1);
  // CommandXboxController test = new CommandXboxController(3);


  Trigger shotReady = new Trigger(() -> swerve.readyToShoot() && hood.readyToShoot() && shooter.readyToShoot());

  SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureAneeshBindings();
    configureEllisBindings();
    // configureTestBindings();
    configurAutoChooser(); 
  }

  private void configurAutoChooser(){
    // Register Named Commands
    NamedCommands.registerCommand("Tracking spin up", new SpinUpCommand(hood, shooter, swerve::getShooterPose, ShotType.Hub));
    NamedCommands.registerCommand("Rotate to tag", new DriveCommand(swerve, () -> 0.0, () -> 0.0, () -> 0.0, () -> true));
    NamedCommands.registerCommand("Feed", new StartEndCommand(
          () -> {feeder.setKickerVoltage(12); feeder.setConveyorVoltage(10);}, 
          () -> {feeder.setKickerVoltage(0); feeder
            .setConveyorVoltage(0);}
        ));
    NamedCommands.registerCommand("Feed when ready", 
      Commands.waitUntil(shotReady::getAsBoolean).andThen(feeder.feed())
    );
  

    NamedCommands.registerCommand("Intake D&R", intake.intake().alongWith(feeder.feed(0, -2)));
    NamedCommands.registerCommand("Agitate intake", intake.agitate(2));
    
    boolean isCompetition = false;

    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream) -> isCompetition ? 
      stream.filter(auto -> auto.getName().startsWith("comp")) : 
      stream
    );

    SmartDashboard.putData("Auto Chooser", autoChooser);
    autoChooser.addOption("8B Preload", fallBackAuto());
  }

  private Command fallBackAuto(){
    return (new SpinUpCommand(hood, shooter, swerve::getShooterPose, ShotType.Hub)).alongWith(
      (new DriveCommand(swerve, () -> -1.0, () -> 0.0, () -> 0.0, () -> true)).withTimeout(0.5).andThen(
      (new DriveCommand(swerve, () -> 0.0, () -> 0.0, () -> 0.0, () -> true)).onlyWhile(shotReady.negate())
      .andThen(
        feeder.feed() 
      )
      )
    );
  }

  private void configureAneeshBindings(){
    neeshneesh.rightTrigger().whileTrue(Rumbler.rumble(0.75, neeshneesh));
    Rumbler.bindTransitionRumbles(neeshneesh);

    //Feeder
    neeshneesh.rightBumper().and(shotReady.or(neeshneesh.rightTrigger())).whileTrue(feeder.feed());
    neeshneesh.leftBumper().whileTrue(feeder.feed(-8, -12));

    //Shooter and Hood
    neeshneesh.b().whileTrue(new SpinUpCommand(hood, shooter, swerve::getShooterPose, ShotType.FixedBumper));
    neeshneesh.a().whileTrue(intake.agitate(2));
    neeshneesh.x().whileTrue(new SpinUpCommand(hood, shooter, swerve::getShooterPose, ShotType.Automatic));   

    //Change Hood Angle
    neeshneesh.povUp().onTrue(Commands.runOnce(() -> SpinUpCommand.incrementOffset(0.1)));
    neeshneesh.povDown().onTrue(Commands.runOnce(() -> SpinUpCommand.incrementOffset(-0.1)));
    neeshneesh.povRight().or(neeshneesh.povLeft()).onTrue(Commands.runOnce(SpinUpCommand::resetOffset));

    // Intake backup
    neeshneesh.y().whileTrue(intake.manualPivotVoltage(1));  
    neeshneesh.leftTrigger().whileTrue(intake.manualPivotVoltage(-1));                                           
  }

  private void configureEllisBindings(){
    // Swerve
    swerve.setDefaultCommand(new DriveCommand(swerve, () -> -smellisEllis.getLeftY(), () -> -smellisEllis.getLeftX(), () -> -smellisEllis.getRightX(), () -> smellisEllis.getRightTriggerAxis() > 0.1));      
    smellisEllis.povUp().onTrue(Commands.runOnce(swerve::resetHeading));
    smellisEllis.a().whileTrue(swerve.crossCommand());



    // Intake backup
    smellisEllis.povUp().whileTrue(intake.manualPivotVoltage(1));  
    smellisEllis.povDown().whileTrue(intake.manualPivotVoltage(-1));


    //Intake regular
    smellisEllis.rightBumper().whileTrue(intake.intake().alongWith(feeder.feed(0, -2)));
    
   
    smellisEllis.leftBumper().whileTrue(intake.extake()
        .alongWith(feeder.feed(-FeederConstants.kConveyorVoltage, 0)));

    smellisEllis.x().onTrue(intake.setPositionCommand(IntakeConstants.kFramePerimStow));
    smellisEllis.y().onTrue(intake.setPositionCommand(IntakeConstants.kZeroPosition));
  }

  // private void configureTestBindings(){
  //   test.rightBumper().whileTrue(SpinUpCommand.fixedShot(ShotCalculator.kTestShot, hood, shooter));
  //   test.leftBumper().whileTrue(feeder.feed(7, -7));
  //   test.x().whileTrue(feeder.applyKickerVoltage(-7));
  //   test.b().whileTrue(feeder.applyKickerVoltage(7));
  // }
 
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();  
  }
}
