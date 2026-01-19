package frc.robot.SwerveSubsystem;

public class Swerve extends SubsystemBase {
      
  private final SwerveModule frontLeft = new SwerveModule(
      ModuleConstants.kFrontLeftDriveMotorID,
      ModuleConstants.kFrontLeftTurnMotorID,
      ModuleConstants.kFrontLeftEncoderID,
      ModuleConstants.kFrontLeftAbsoluteEncoderOffset,
      ModuleConstants.kFrontLeftAbsoluteEncoderReversed
    );

    private final SwerveModule frontRight = new SwerveModule(
      ModuleConstants.kFrontRightDriveMotorID,
      ModuleConstants.kFrontRightTurnMotorID,
      ModuleConstants.kFrontRightEncoderID,
      ModuleConstants.kFrontRightAbsoluteEncoderOffset,
      ModuleConstants.kFrontRightAbsoluteEncoderReversed
    );

    private final SwerveModule backLeft = new SwerveModule(
      ModuleConstants.kBackLeftDriveMotorID,
      ModuleConstants.kBackLeftTurnMotorID,
      ModuleConstants.kBackLeftEncoderID,
      ModuleConstants.kBackLeftAbsoluteEncoderOffset,
      ModuleConstants.kBackLeftAbsoluteEncoderReversed
    );

    private final SwerveModule backRight = new SwerveModule(
      ModuleConstants.kBackRightDriveMotorID,
      ModuleConstants.kBackRightTurnMotorID,
      ModuleConstants.kBackRightEncoderID,
      ModuleConstants.kBackRightAbsoluteEncoderOffset,
      ModuleConstants.kBackRightAbsoluteEncoderReversed
    );

    SwerveModule[] modules = {frontLeft, frontRight, backLeft, backRight};

    Pigeon2 pigeon = new Pigeon2(0);



  public Swerve() {
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e) {
        
      }
    }).start();
  }

  public void zeroHeading(){
    pigeon.reset();
  }

  public double getHeading(){
    return pigeon.getYaw().getValueAsDouble();
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
    frontLeft.desiredState(desiredStates[0]);
    frontRight.desiredState(desiredStates[1]);
    backLeft.desiredState(desiredStates[2]);
    backRight.desiredState(desiredStates[3]);
  }
}
