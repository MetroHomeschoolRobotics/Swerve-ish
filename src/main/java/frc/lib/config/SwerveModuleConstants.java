package frc.lib.config;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
  public final int driveMotorID;
  public final int angleMotorID;
  public final int angleEncoder0;
  public final int angleEncoder1;
  public final int driveEncoder0;
  public final int driveEncoder1;
  public final Rotation2d angleOffset;
  public final double angleMotorSpeedOffset;

  /**
   * Swerve Module Constants to be used when creating swerve modules.
   *
   * @param driveMotorID
   * @param angleMotorID
   * @param canCoderID
   * @param angleOffset
   */
  public SwerveModuleConstants(
      int driveMotorID, int angleMotorID, int angleEncoder0, int angleEncoder1, int driveEncoder0, int driveEncoder1, Rotation2d angleOffset, double angleMotorSpeedOffset) {
    this.driveMotorID = driveMotorID;
    this.angleMotorID = angleMotorID;
    this.angleEncoder0 = angleEncoder0;
    this.angleEncoder1 = angleEncoder1;
    this.driveEncoder0 = driveEncoder0;
    this.driveEncoder1 = driveEncoder1;
    this.angleOffset = angleOffset;
    this.angleMotorSpeedOffset = angleMotorSpeedOffset;
  }
}
