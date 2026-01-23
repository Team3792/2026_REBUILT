package frc.robot.SwerveSubsystem;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
      
  private final SwerveModule frontLeft = new SwerveModule(ModuleConstants.kFrontLeft, "front left");
    private final SwerveModule frontRight = new SwerveModule(ModuleConstants.kFrontRight, "front right");
    private final SwerveModule backLeft = new SwerveModule(ModuleConstants.kBackLeft, "back left");
    private final SwerveModule backRight = new SwerveModule(ModuleConstants.kBackRight, "back right");

    SwerveModule[] modules = {frontLeft, frontRight, backLeft, backRight};

    Pigeon2 pigeon = new Pigeon2(0);
    
  public Swerve() {
    zeroHeading();


  }

  public void zeroHeading(){
    pigeon.reset();
  }

  public double getHeading(){
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

  public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, SwerveConstants.kMaxSpeedMetersPerSecond);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(targetStates[i]);
    }
  }
}
