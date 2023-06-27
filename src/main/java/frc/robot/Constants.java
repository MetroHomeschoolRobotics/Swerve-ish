package frc.robot;
//import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;

public final class Constants {

  public static final class Swerve {
    public static final double stickDeadband = 0.1;
    public static final double anglePPR = 764; //These are a rough estimate to calibrate the encoders
    public static final double distancePPR = 0.000039291;
    public static final int pigeonID = 6;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(14);//21.73
    public static final double wheelBase = Units.inchesToMeters(14);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;
//TODO fix gear ratio
    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 80;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor =
        (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5; // meters per second
    public static final double maxAngularVelocity = 11.5;

    /* Neutral Modes */
    //public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    //public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = false;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 0;//4
      public static final int angleMotorID = 1;
      public static final int angleEncoder0 = 0;
      public static final int angleEncoder1 = 1;
      public static final int driveEncoder0 = 10;
      public static final int driveEncoder1 = 11;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);//327.48046875
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, angleEncoder0, angleEncoder1, driveEncoder0, driveEncoder1, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 2;//14
      public static final int angleMotorID = 3;//13
      public static final int angleEncoder0 = 2;//2
      public static final int angleEncoder1 = 3;
      public static final int driveEncoder0 = 12;
      public static final int driveEncoder1 = 13;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);//286.34765625
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, angleEncoder0, angleEncoder1, driveEncoder0, driveEncoder1, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 4;//2
      public static final int angleMotorID = 5;//1
      public static final int angleEncoder0 = 4;//2
      public static final int angleEncoder1 = 5;//3
      public static final int driveEncoder0 = 18;
      public static final int driveEncoder1 = 19;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);//55.01953125
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, angleEncoder0, angleEncoder1, driveEncoder0, driveEncoder1, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 6;
      public static final int angleMotorID = 7;
      public static final int angleEncoder0 = 6;
      public static final int angleEncoder1 = 7;      
      public static final int driveEncoder0 = 20;
      public static final int driveEncoder1 = 21;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);//67.939453125
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, angleEncoder0, angleEncoder1, driveEncoder0, driveEncoder1, angleOffset);
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

}
