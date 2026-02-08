package frc.robot.SwerveSubsystem;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Localization.AntPose;

public class Swerve extends SubsystemBase {
      
  static SwerveModule frontLeft = new SwerveModule(ModuleConstants.kFrontLeft, "front left");
  static SwerveModule frontRight = new SwerveModule(ModuleConstants.kFrontRight, "front right");
  static SwerveModule backLeft = new SwerveModule(ModuleConstants.kBackLeft, "back left");
  static SwerveModule backRight = new SwerveModule(ModuleConstants.kBackRight, "back right");

  SwerveModule[] modules = {frontLeft, frontRight, backLeft, backRight};

  public static Pigeon2 pigeon = new Pigeon2(0);

  AntPose poseEstimator = new AntPose(
    SwerveConstants.kDriveKinematics, 
    getRotation2dHeading(), 
    getModulePositions(), 
    new Pose2d(), 
    null,
    null);

  HolonomicDriveController swerveController = new HolonomicDriveController(SwerveConstants.kTranslationController, 
                                        SwerveConstants.kTranslationController, SwerveConstants.kRotationController);
    
  
  public Swerve() {
    zeroHeading();
  }

  public void zeroHeading(){
    pigeon.reset();
  }

  public static double getHeading(){
    return pigeon.getYaw().getValueAsDouble();
  }

  public void resetHeading(double yawDegrees){
    pigeon.setYaw(yawDegrees);
  }

  public Rotation2d getRotation2dHeading(){
    return Rotation2d.fromDegrees(getHeading());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Robot Heading", getHeading());
  }

  public void stopModules(){
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public static SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition frontLeftPos = frontLeft.getPosition();
    SwerveModulePosition frontRightPos = frontRight.getPosition();
    SwerveModulePosition backLeftPos = backLeft.getPosition();
    SwerveModulePosition backRightPos = backRight.getPosition();
    SwerveModulePosition[] modulePositions = {frontLeftPos, frontRightPos, backLeftPos, backRightPos};
    
    return modulePositions;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.kMaxSpeedMetersPerSecond);
    frontLeft.setState(desiredStates[0]);
    frontRight.setState(desiredStates[1]);
    backLeft.setState(desiredStates[2]);
    backRight.setState(desiredStates[3]);
  }

  public void drive(ChassisSpeeds speeds, Rotation2d rotation){
    speeds = ChassisSpeeds.discretize(speeds, 0.002);
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rotation);

    SwerveModuleState[] desiredStates = SwerveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    setModuleStates(desiredStates);
  }

  //Use for testitng
  public void turnToTag(double tagYaw) {
    ProfiledPIDController pid = new ProfiledPIDController(0.01, 0, 0, new TrapezoidProfile.Constraints(SwerveConstants.kMaxOmega, SwerveConstants.kMaxOmega));
    double rotationSpeed = pid.calculate(getHeading(), tagYaw);
    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, rotationSpeed);
    drive(speeds, getRotation2dHeading());
  }

  public void alignToTag(Pose2d tagPose) {
    // Calculate new target pose with offset
    Pose2d targetPose = tagPose.transformBy(new Transform2d(0.5, 0, new Rotation2d(0)));

    Pose2d robotPose = poseEstimator.getEstimatedPosition();
    ChassisSpeeds speeds = swerveController.calculate(robotPose, targetPose, 0, targetPose.getRotation());
    drive(speeds, getRotation2dHeading());
  }

  public Command turnToTagCommand(double tagYaw) {
    return Commands.run(() -> turnToTag(tagYaw));
  }

  public Command alignToTagCommand(Pose2d tagPose){
    return Commands.run(() -> alignToTag(tagPose));
  }
}
