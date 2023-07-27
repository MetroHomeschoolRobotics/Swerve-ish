package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.math.OnboardModuleState;
import frc.robot.Constants;

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  private Talon angleMotor;
  private Talon driveMotor;

  private Encoder driveEncoder;
  //private Encoder integratedAngleEncoder;
  private Encoder angleEncoder;

  private final PIDController driveController;
  private final PIDController angleController;

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    angleEncoder = new Encoder(moduleConstants.angleEncoder0, moduleConstants.angleEncoder1);
    //configAngleEncoder();

    /* Angle Motor Config */
    angleMotor = new Talon(moduleConstants.angleMotorID);
    angleController = new PIDController(Constants.Swerve.angleKP,Constants.Swerve.angleKI,Constants.Swerve.angleKD);
    resetToAbsolute();
    //configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new Talon(moduleConstants.driveMotorID);
    driveEncoder = new Encoder(moduleConstants.driveEncoder0,moduleConstants.driveEncoder1);
    driveController = new PIDController(Constants.Swerve.driveKP,Constants.Swerve.driveKI,Constants.Swerve.driveKD);
    driveEncoder.reset();
    //configDriveMotor();

    lastAngle = getState().angle;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // Custom optimize command, since default WPILib optimize assumes continuous controller which
    // REV and CTRE are not
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  private void resetToAbsolute() {
    double absolutePosition = getAngle().getDegrees() - angleOffset.getDegrees();
    angleEncoder.setDistancePerPulse(absolutePosition*0.000886889);
  }

  //private void configAngleEncoder() {
    //angleEncoder.configFactoryDefault();
    //CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
    //angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
  //}
/* 
  private void configAngleMotor() {
    angleMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    angleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
    angleMotor.setInverted(Constants.Swerve.angleInvert);
    angleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
    integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
    angleController.setP(Constants.Swerve.angleKP);
    angleController.setI(Constants.Swerve.angleKI);
    angleController.setD(Constants.Swerve.angleKD);
    angleController.setFF(Constants.Swerve.angleKFF);
    angleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    angleMotor.burnFlash();
    resetToAbsolute();
  }
 */
  //private void configDriveMotor() {
    //driveMotor.restoreFactoryDefaults();
    //CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    //driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
    //driveMotor.setInverted(Constants.Swerve.driveInvert);
    //driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
    //driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
    //driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
    //driveController.setP(Constants.Swerve.angleKP);
    //driveController.setI(Constants.Swerve.angleKI);
    //driveController.setD(Constants.Swerve.angleKD);
    //driveController.setFF(Constants.Swerve.angleKFF);
    //driveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    //driveMotor.burnFlash();
    //driveEncoder.setPosition(0.0);
  //}

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      driveMotor.set(percentOutput);
    } else {
      //driveController.setReference(desiredState.speedMetersPerSecond,ControlType.kVelocity,0,feedforward.calculate(desiredState.speedMetersPerSecond));
      driveMotor.set(feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle;

    //angleController.setReference(angle.getDegrees(), ControlType.kPosition);
    angleMotor.set(desiredState.speedMetersPerSecond/Constants.Swerve.maxSpeed);
    lastAngle = angle;
  }

  public Rotation2d getAngle() {
    double angle = Math.IEEEremainder(angleEncoder.get(), Constants.Swerve.anglePPR)/Constants.Swerve.anglePPR *360;
        return Rotation2d.fromDegrees(angle);
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getDistance());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveEncoder.get()*Constants.Swerve.distancePPR, getAngle());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getRate()*Constants.Swerve.distancePPR, getAngle());
  }

}