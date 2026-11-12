package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.SpinUpCommand;

import frc.robot.Commands.SpinUpCommand.ShotType;
import frc.robot.Subsystems.FeederSubsystem.Feeder;
import frc.robot.Subsystems.FeederSubsystem.FeederConstants;
import frc.robot.Subsystems.IntakeSubsystem.Intake;
import frc.robot.Subsystems.IntakeSubsystem.IntakeConstants;
import frc.robot.Subsystems.ShooterSubsystem.Hood.*;
import frc.robot.Subsystems.ShooterSubsystem.Shooter.*;
import frc.robot.Subsystems.SwerveSubsystem.DriveCommand;
import frc.robot.Subsystems.SwerveSubsystem.Swerve;
import frc.robot.util.HardwareMap;
import frc.robot.util.Rumbler;

public class RobotContainer {
  //Subsystems
  Feeder feeder = new Feeder();
  Intake intake = new Intake();
  Swerve swerve = new Swerve();
  Shooter shooter = new Shooter();
  Hood hood = new Hood();

  //Controllers
  CommandPS5Controller driver = new CommandPS5Controller(HardwareMap.kDrivePort);
  CommandXboxController operator = new CommandXboxController(HardwareMap.kOperatorPort);
  CommandPS5Controller outreach = new CommandPS5Controller(HardwareMap.kOutreachPort);

  //Triggers
  Trigger shotReady = new Trigger(() -> swerve.readyToShoot() && hood.readyToShoot() && shooter.readyToShoot());

  SendableChooser<Command> autoChooser;

  public RobotContainer() {
    //configureOutreachBindings();
    configureCleanOutreachBindings();
    //configureOperatorBindings();
    //configureDriverBindings();
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
      Commands.waitUntil(shotReady::getAsBoolean).andThen(
        feeder.feed().alongWith(
        Commands.waitSeconds(1).andThen(intake.agitate(1))
      )
      )
    );
    NamedCommands.registerCommand("Intake D&R", intake.intake().alongWith(feeder.feed(0, -2)));
    NamedCommands.registerCommand("Agitate intake", intake.agitate(3));
    
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

  private void configureOutreachBindings(){
    //Driving: joysticks for driving + right trigger for heading lock
    swerve.setDefaultCommand(new DriveCommand(swerve, () -> -outreach.getLeftY(), () -> -outreach.getLeftX(), () -> -outreach.getRightX(), () -> outreach.getR2Axis() > 0.1));      
    driver.povUp().onTrue(Commands.runOnce(swerve::resetHeading));
    driver.L2().whileTrue(swerve.crossCommand());

    //Intake: right bumper for intake and deploy + left bumper for extake + a for frame perimeter stow + pov down for zero position stow
    outreach.R1().whileTrue(intake.intake().alongWith(feeder.feed(0, -2)));
    outreach.L1().whileTrue(intake.extake()
        .alongWith(feeder.feed(-FeederConstants.kConveyorVoltage, 0)));
    outreach.square().onTrue(intake.setPositionCommand(IntakeConstants.kFramePerimStow));
    outreach.triangle().onTrue(intake.setPositionCommand(IntakeConstants.kZeroPosition));

    //Feeder: right stick for feeding to shooter + left stick for extaking 
    outreach.L2().and(shotReady.or(outreach.L3())).whileTrue(feeder.feed());
    outreach.R3().whileTrue(feeder.feed(-8, -12));

    //Shooting: right trigger for spin up
    //outreach.b().whileTrue(new SpinUpCommand(hood, shooter, swerve::getShooterPose, ShotType.FixedBumper));
    outreach.R2().whileTrue(new SpinUpCommand(hood, shooter, swerve::getShooterPose, ShotType.Automatic));   
    //outreach.y().whileTrue(new SpinUpCommand(hood, shooter, swerve::getShooterPose, ShotType.FixedFeed));

    //Intake agitate: a for agitating
    outreach.L2().whileTrue(intake.agitate(1.5));
  }

    private void configureCleanOutreachBindings(){
    //Driving: joysticks for driving + right trigger for heading lock
    swerve.setDefaultCommand(new DriveCommand(swerve, () -> -outreach.getLeftY(), () -> -outreach.getLeftX(), () -> -outreach.getRightX(), () -> outreach.getR2Axis() > 0.1));      
    outreach.povUp().onTrue(Commands.runOnce(swerve::resetHeading));
    outreach.L2().whileTrue(swerve.crossCommand());

    //Intake: right bumper for intake and deploy + left bumper for extake + a for frame perimeter stow + pov down for zero position stow
    outreach.R1().whileTrue(intake.intake().alongWith(feeder.feed(0, -2)));
    outreach.L1().whileTrue(intake.extake()
        .alongWith(feeder.feed(-FeederConstants.kConveyorVoltage, 0)));
    outreach.square().onTrue(intake.setPositionCommand(IntakeConstants.kFramePerimStow));
    outreach.triangle().onTrue(intake.setPositionCommand(IntakeConstants.kZeroPosition));

    //Feeder: left trigger for feeding to shooter + left stick for extaking 
    outreach.L2().and(shotReady.or(outreach.L3())).whileTrue(feeder.feed());
    outreach.R3().whileTrue(feeder.feed(-8, -12));

    //Shooting: right trigger for spin up
    outreach.R2().whileTrue(new SpinUpCommand(hood, shooter, swerve::getShooterPose, ShotType.Automatic));   

    //Intake agitate: a for agitating
    outreach.L2().whileTrue(intake.agitate(1.5));
  }

  // private void configureOperatorBindings(){
  //   //Feedback
  //   Rumbler.bindTransitionRumbles(operator);

  //   //Feeder
  //   operator.rightBumper().and(shotReady.or(operator.rightTrigger())).whileTrue(feeder.feed());
  //   operator.leftBumper().whileTrue(feeder.feed(-8, -12));

  //   //Shooting
  //   operator.b().whileTrue(new SpinUpCommand(hood, shooter, swerve::getShooterPose, ShotType.FixedBumper));
  //   operator.x().whileTrue(new SpinUpCommand(hood, shooter, swerve::getShooterPose, ShotType.Automatic));   
  //   operator.y().whileTrue(new SpinUpCommand(hood, shooter, swerve::getShooterPose, ShotType.FixedFeed));

  //   //Incriment Shot Distance Override
  //   operator.povUp().onTrue(Commands.runOnce(() -> SpinUpCommand.incrementOffset(0.1)));
  //   operator.povDown().onTrue(Commands.runOnce(() -> SpinUpCommand.incrementOffset(-0.1)));
  //   operator.povRight().or(operator.povLeft()).onTrue(Commands.runOnce(SpinUpCommand::resetOffset)); 
    
  //   //Intake Agitate (drive or operator)
  //    operator.a().or(driver.cross()).whileTrue(intake.agitate(1.5));
  // }

  // private void configureDriverBindings(){
  //   //Swerve
  //   swerve.setDefaultCommand(new DriveCommand(swerve, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX(), () -> driver.getR2Axis() > 0.1));      
  //   driver.povUp().onTrue(Commands.runOnce(swerve::resetHeading));
  //   driver.L2().whileTrue(swerve.crossCommand());

  //   //Intake regular
  //   driver.R1().whileTrue(intake.intake().alongWith(feeder.feed(0, -2)));
  //   driver.L1().whileTrue(intake.extake()
  //       .alongWith(feeder.feed(-FeederConstants.kConveyorVoltage, 0)));
  //   driver.square().onTrue(intake.setPositionCommand(IntakeConstants.kFramePerimStow));
  //   driver.circle().onTrue(intake.setPositionCommand(IntakeConstants.kZeroPosition));

  //   //Intake backup
  //   driver.povUp().whileTrue(intake.manualPivotVoltage(1));  
  //   driver.povDown().whileTrue(intake.manualPivotVoltage(-1));
  // }
 
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();  
  }
}
