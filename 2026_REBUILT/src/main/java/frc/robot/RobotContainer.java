// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.ShootCommand;
import frc.robot.Localization.Vision;
import frc.robot.Managers.FieldManager;
import frc.robot.Managers.GameManager;
import frc.robot.Managers.ZoneManager;
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

public class RobotContainer {

  Feeder feeder = new Feeder();
  Intake intake = new Intake();
  Climb climb = new Climb();
  Swerve swerve = new Swerve();
  Shooter shooter = new Shooter();
  Hood hood = new Hood();

  CommandXboxController ellis = new CommandXboxController(0);
  CommandXboxController aneesh = new CommandXboxController(1);

  Trigger transitionShift = new Trigger(() -> GameManager.atTransitionShift());
  Trigger shiftOne = new Trigger(() -> GameManager.atShiftOne());
  Trigger shiftTwo = new Trigger(() -> GameManager.atShiftTwo());
  Trigger shiftThree = new Trigger(() -> GameManager.atShiftThree());
  Trigger shiftFour = new Trigger(() -> GameManager.atShiftFour());
  Trigger endGame = new Trigger(() -> GameManager.atEndGame());

  Trigger shotReady = new Trigger(() -> swerve.readyToShoot() && hood.readyToShoot() && shooter.readyToShoot());

  SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureAneeshBindings();
    configureEllisBindings();

    //Register Named Commands
   
    //intervals of 5 cms
    // for (int i = 1; i < 101; i++){
    //   double distance = i * 5;
    //   double rps = shooter.getTargetRPS(distance);
    //   double angle = hood.targetHoodAngle(distance);
    //   String name = distance + " cm shot";
    //   NamedCommands.registerCommand(name, shooter.setFlywheelRPS(rps).alongWith(hood.setHoodAngle(angle).alongWith(feeder.runFeeder(12, 8))));
    // }
    


      configurAutoChooser();
    }

  private void configurAutoChooser(){
    // NamedCommands.registerCommand("Deploy Intake", intake.intake().alongWith(feeder.runConveyor(FeederConstants.kConveyorVoltage)));
    // NamedCommands.registerCommand("Stow Intake", intake.pivotToAngle(IntakeConstants.kStowOut));
    
    // // Used name commands
    // NamedCommands.registerCommand("Tracking spin up", new ShootCommand(hood, shooter, swerve::getShooterPose));
    //NamedCommands.registerCommand("test", new ShootCommand(hood, shooter, swerve::getShooterPose).withTimeout(5));
    //NamedCommands.registerCommand("testing", Commands.run(() -> shooter.setRPS(30)));
    NamedCommands.registerCommand("Rotate to tag", new DriveCommand(swerve, () -> 0.0, () -> 0.0, () -> 0.0, () -> true));
    NamedCommands.registerCommand("Feed", new StartEndCommand(
          () -> {feeder.setKickerVoltage(12); feeder.setConveyorVoltage(10);}, 
          () -> {feeder.setKickerVoltage(0); feeder
            .setConveyorVoltage(0);}
        ));
    NamedCommands.registerCommand("Feed when ready", Commands.waitUntil(shotReady::getAsBoolean).andThen(new StartEndCommand(
          () -> {feeder.setKickerVoltage(12); feeder.setConveyorVoltage(10);}, 
          () -> {feeder.setKickerVoltage(0); feeder.setConveyorVoltage(0);}
        )));

    System.out.println(NamedCommands.hasCommand("test"));
    
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
    return (new ShootCommand(hood, shooter, swerve::getShooterPose)).alongWith(
      (new DriveCommand(swerve, () -> -1.0, () -> 0.0, () -> 0.0, () -> true)).withTimeout(0.5).andThen(
      (new DriveCommand(swerve, () -> 0.0, () -> 0.0, () -> 0.0, () -> true)).onlyWhile(shotReady.negate())
      .andThen(
        new StartEndCommand(
          () -> {feeder.setKickerVoltage(12); feeder.setConveyorVoltage(10);}, 
          () -> {feeder.setKickerVoltage(0); feeder.setConveyorVoltage(0);}
        ) 
      )
      )
    );
  }

  private void configureAneeshBindings(){
    //nearTrench.whileTrue(rumble(0.7, 5));

    aneesh.rightTrigger().whileTrue(rumble(0.75, aneesh));

    transitionShift.onTrue(rumble(1, 4, aneesh));
    shiftOne.onTrue(rumble(1, 4, aneesh));
    shiftTwo.onTrue(rumble(1, 4, aneesh));
    shiftThree.onTrue(rumble(1, 4, aneesh));
    shiftFour.onTrue(rumble(1, 4, aneesh));
    endGame.onTrue(rumble(1, 4, aneesh));


    //Feeder
    aneesh.rightBumper().and(shotReady.or(aneesh.rightTrigger()))
    .whileTrue(
      new StartEndCommand(
        () -> {feeder.setKickerVoltage(12); feeder.setConveyorVoltage(8);}, 
        () -> {feeder.setKickerVoltage(0); feeder.setConveyorVoltage(0);}
      )
    );
    
    
    aneesh.leftBumper().whileTrue(
      new StartEndCommand(
        () -> {feeder.setKickerVoltage(-12); feeder.setConveyorVoltage(-8);}, 
        () -> {feeder.setKickerVoltage(0); feeder.setConveyorVoltage(0);}
      )
    );

    //Shooter and Hood
    //Hub Shot
    aneesh.b().whileTrue(ShootCommand.fixedShot(ShotCalculator.kBumperShot, hood, shooter));
    aneesh.a().whileTrue(ShootCommand.fixedShot(ShotCalculator.kFeedShot, hood, shooter));
    // aneesh.b().whileTrue(Commands.startEnd(() -> shooter.setRPS(50), 
    //                             () -> shooter.setVoltage(0)).alongWith(Commands.startEnd(() -> hood.setAngle(15), 
    //                               () -> hood.setAngle(12))));

    // aneesh.b().whileTrue(Commands.startEnd(
    //   () -> shooter.setRPS(ShotCalculator.kBumperShot.omegaRPS())
    // ));

    // aneesh.a().whileTrue(Commands.startEnd(() -> shooter.setRPS(50), () -> shooter.setVoltage(0))
    //       .alongWith(Commands.startEnd(() -> hood.setAngle(15), () -> hood.setAngle(12))));

    //Changeable Shot                  
    SmartDashboard.putNumber("Set RPS", 45);
    SmartDashboard.putNumber("Set Hood Angle", 20);  

    aneesh.x().whileTrue(new ShootCommand(hood, shooter, swerve::getShooterPose));          
    //aneesh.y().onTrue(intake.zeroIntake());                           

    //Change Hood Angle
    aneesh.povUp().onTrue(hood.incrementHoodAngle(HoodConstants.kAngleIncrement));
    aneesh.povDown().onTrue(hood.incrementHoodAngle(-HoodConstants.kAngleIncrement));
    aneesh.povRight().or(aneesh.povLeft()).onTrue(hood.resetIncrement());
    
    //Change Shooter RPS
    // aneesh.povRight().onTrue(shooter.increaseRPS()); //Increases by 2 RPS
    // aneesh.povLeft().onTrue(shooter.decreaseRPS()); //Decreases by 1 RPS

    // Intake backup
    aneesh.y().whileTrue(intake.movePivot(1));  
    aneesh.leftTrigger().whileTrue(intake.movePivot(-1));                         
  }

  private void configureEllisBindings(){
    transitionShift.onTrue(rumble(1, 4, ellis));
    shiftOne.onTrue(rumble(1, 4, ellis));
    shiftTwo.onTrue(rumble(1, 4, ellis));
    shiftThree.onTrue(rumble(1, 4, ellis));
    shiftFour.onTrue(rumble(1, 4, ellis));
    endGame.onTrue(rumble(1, 4, ellis));

    // Swerve
    swerve.setDefaultCommand(new DriveCommand(swerve, () -> -ellis.getLeftY(), () -> -ellis.getLeftX(), () -> -ellis.getRightX(), () -> ellis.getRightTriggerAxis() > 0.1));      
    ellis.povUp().onTrue(Commands.runOnce(swerve::resetHeading));
    ellis.leftTrigger().whileTrue(swerve.crossCommand());
   

    // Intake backup
    ellis.povUp().whileTrue(intake.movePivot(1));  
    ellis.povDown().whileTrue(intake.movePivot(-1));

    //Intake regular
    ellis.rightBumper().whileTrue(
      Commands.startEnd(() -> {
        intake.setPosition(IntakeConstants.kIntakePosition);
        intake.setDriveVoltage(IntakeConstants.kIntakeVoltage);}, () ->
          intake.setDriveVoltage(0),intake)
        .alongWith(feeder.runConveyor(FeederConstants.kConveyorVoltage)));

    ellis.leftBumper().whileTrue(
      Commands.startEnd(() -> {
        intake.setPosition(IntakeConstants.kIntakePosition);
        intake.setDriveVoltage(-IntakeConstants.kIntakeVoltage);}, 
        () -> intake.setDriveVoltage(0),
        intake)
        .alongWith(feeder.runConveyor(-FeederConstants.kConveyorVoltage)));

    ellis.x().onTrue(Commands.runOnce(() -> intake.setPosition(IntakeConstants.kFramePerimStow), intake));
    ellis.y().onTrue(Commands.runOnce(() -> intake.setPosition(IntakeConstants.kZeroPosition)));

    // alongWith(Commands.startEnd(() -> feeder.setConveyorVoltage(FeederConstants.kConveyorVoltage), () -> feeder.setConveyorVoltage(0)))
    // .alongWith(Commands.startEnd(() -> intake.setPosition(IntakeConstants.kIntakePosition), () -> intake.setPosition(IntakeConstants.kStowedPosition))));
    //   //outtake
    
    //ellis.rightTrigger().whileTrue(Commands.startEnd(() -> intake.setPosition(IntakeConstants.kIntakePosition), 
                        //() -> intake.setPosition(IntakeConstants.kStowedPosition)));

    //make a boolean that shows if the intake is down
  }
 

  // intensity on a scale of 0-1 and duration in seconds
  private Command rumble(double intensity, double duration, CommandXboxController controller){
      return Commands.startEnd(() -> controller.setRumble(RumbleType.kBothRumble, intensity), () -> controller.setRumble(RumbleType.kBothRumble, 0)).withTimeout(duration);
  }

  private Command rumble(double intensity, CommandXboxController controller){
    return Commands.startEnd(() -> controller.setRumble(RumbleType.kBothRumble, intensity), () -> controller.setRumble(RumbleType.kBothRumble, 0));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();  
  }
}
