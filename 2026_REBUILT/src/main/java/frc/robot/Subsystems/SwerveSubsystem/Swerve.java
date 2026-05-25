package frc.robot.Subsystems.SwerveSubsystem;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.SpinUpCommand.ShotType;
import frc.robot.Localization.AntPoseEstimator;
import frc.robot.Localization.VisionConstants;
import frc.robot.Managers.FieldManager;
import frc.robot.Managers.GameManager;
import frc.robot.util.HardwareMap;

public class Swerve extends SubsystemBase {
  SwerveModule frontLeft = new SwerveModule(ModuleConstants.kFrontLeft, "front left");
  SwerveModule frontRight = new SwerveModule(ModuleConstants.kFrontRight, "front right");
  SwerveModule backLeft = new SwerveModule(ModuleConstants.kBackLeft, "back left");
  SwerveModule backRight = new SwerveModule(ModuleConstants.kBackRight, "back right");
  SwerveModule[] modules = {frontLeft, frontRight, backLeft, backRight};
  
  Pigeon2 pigeon = new Pigeon2(HardwareMap.kPigeon.id());
  ChassisSpeeds lastRobotRelativeSpeeds;
  double currentTimestamp = 0.0;
  AntPoseEstimator antPose;
  Field2d field = new Field2d();
  
  public Swerve() {
    antPose = new AntPoseEstimator(SwerveConstants.kDriveKinematics, getPigeonHeading(), getModulePositions(), new Pose2d());
    resetHeading();
    SmartDashboard.putData("Shooter pose", field);

    configureAutoBuilder();
  }

  public void resetTurnEncoders(){
    for (SwerveModule mod : modules){
      mod.resetEncoders();
    }
  }

  public Rotation2d getPigeonHeading(){
    return pigeon.getRotation2d();
  }

  public Rotation2d getPoseHeading(){
    return antPose.getEstimatedPosition().getRotation();
  }

  private void configureAutoBuilder() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      config = new RobotConfig(0, 0, new ModuleConfig(0, 0, 0, new DCMotor(0, 0, 0, 0, 0, 4), 0, 4));
      e.printStackTrace();
    }

    AutoBuilder.configure(
      this::getPose,
      this::resetPose,
      this::getRobotRelativeSpeeds,
      (speeds, feedforward) -> drive(speeds),
      new PPHolonomicDriveController(new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
      config,
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()){
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this
    );
  }

  @Override
  public void periodic() {
    //Update Localization
    antPose.updateAll(getPigeonHeading(), getModulePositions());
    printDebug();
  }

  public void printDebug(){
    SmartDashboard.putNumber("Robot Heading (deg)", getPigeonHeading().getDegrees());
    frontLeft.printEncoders();
    frontRight.printEncoders();
    backLeft.printEncoders();
    backRight.printEncoders();

    SmartDashboard.putNumber("Distance to target (m)", FieldManager.getTargetDistance(getShooterPose(), ShotType.Automatic));
    SmartDashboard.putBoolean("Swerve Ready for Shot", readyToShoot());
  }

  public void stopModules(){
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void setTurnAngleAll(double angleRadians){
    frontLeft.setState(new SwerveModuleState(0, new Rotation2d(angleRadians)));
    frontRight.setState(new SwerveModuleState(0, new Rotation2d(angleRadians)));
    backLeft.setState(new SwerveModuleState(0, new Rotation2d(angleRadians)));
    backRight.setState(new SwerveModuleState(0, new Rotation2d(angleRadians)));
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition frontLeftPos = frontLeft.getPosition();
    SwerveModulePosition frontRightPos = frontRight.getPosition();
    SwerveModulePosition backLeftPos = backLeft.getPosition();
    SwerveModulePosition backRightPos = backRight.getPosition();
    SwerveModulePosition[] modulePositions = {frontLeftPos, frontRightPos, backLeftPos, backRightPos};
    
    return modulePositions;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ModuleConstants.kMaxModuleSpeedMetersPerSecond);
    
    for(int i = 0; i < 4; i++)
      modules[i].setState(desiredStates[i]);
  }

  public void drive(ChassisSpeeds speeds) {
    speeds = ChassisSpeeds.discretize(speeds, 0.002);

    SwerveModuleState[] desiredStates = SwerveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    setModuleStates(desiredStates);
  }

  public Command crossCommand(){
    return this.run(() -> {
      frontLeft.setState(new SwerveModuleState(0, new Rotation2d(Math.PI/4)));
      frontRight.setState(new SwerveModuleState(0, new Rotation2d(-Math.PI/4)));
      backLeft.setState(new SwerveModuleState(0, new Rotation2d(-Math.PI/4)));
      backRight.setState(new SwerveModuleState(0, new Rotation2d(Math.PI/4)));
    });
  }

  public void resetPose(Pose2d pose){
    antPose.resetPose(pose);
  }

  //Resets pose heading to be straight forward from driver perspective 
  public void resetHeading(){
    //Flip if red
    Rotation2d heading = (GameManager.getAllianceColor() == Alliance.Red) ? Rotation2d.k180deg : Rotation2d.kZero;
    resetPose(new Pose2d(
      getPose().getTranslation(),
      heading
    ));
  }

  //Returns estimated robot pose
  public Pose2d getPose(){
    return antPose.getEstimatedPosition();
  } 

  //Returns estimated shooter pose with shooter offset
  public Pose2d getShooterPose(){
    Pose2d shooterPose = getPose().transformBy(VisionConstants.kRobotToShooter);
    field.setRobotPose(shooterPose);
    return shooterPose;
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return SwerveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
  }

  //returns array of field relative robot velocity [x,y]
  public double[] getFieldRelativeSpeeds(){
    double robotRelativeVelocityX = getRobotRelativeSpeeds().vxMetersPerSecond;
    double robotRelativeVelocityY = getRobotRelativeSpeeds().vyMetersPerSecond;
    double fieldRelativeVelocityX = robotRelativeVelocityX * getPigeonHeading().getCos() + robotRelativeVelocityY * getPigeonHeading().getSin();
    double fieldRelativeVelocityY = robotRelativeVelocityX * getPigeonHeading().getSin() + robotRelativeVelocityY * getPigeonHeading().getCos();
    double fieldRelativeVelocityTotal = Math.hypot(fieldRelativeVelocityX, fieldRelativeVelocityY);
    double[] fieldRelativeVelocities = {fieldRelativeVelocityX, fieldRelativeVelocityY, fieldRelativeVelocityTotal};
    return fieldRelativeVelocities;
  }

  public boolean readyToShoot(){
    ChassisSpeeds robotRelativeSpeeds = getRobotRelativeSpeeds();
    return Math.hypot(robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond) < SwerveConstants.kZeroTranslationThreshold
      && robotRelativeSpeeds.omegaRadiansPerSecond < SwerveConstants.kZeroOmegaThreshold;
  }
}
