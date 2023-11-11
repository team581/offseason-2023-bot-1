// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.swerve.SwerveCorner;
import frc.robot.swerve.SwerveModuleConstants;

public class Config {
  public static final String SERIAL_NUMBER = System.getenv("serialnum");

  // TODO: Change this to false during events
  public static final boolean IS_DEVELOPMENT = false;

  public static final int SHOULDER_ID = 14;
  public static final int SHOULDER2_ID = 15;
  public static final int PIGEON2_ID = 1;
  public static final int PDP_ID = 0;
  public static final int WRIST_ID = 16;
  public static final int INTAKE_ID = 17;
  public static final int CANDLE_ID = 18;

  public static final Translation2d SWERVE_FRONT_LEFT_LOCATION =
      new Translation2d(0.263525, 0.263525);
  public static final Translation2d SWERVE_FRONT_RIGHT_LOCATION =
      new Translation2d(0.263525, -0.263525);
  public static final Translation2d SWERVE_BACK_LEFT_LOCATION =
      new Translation2d(-0.263525, 0.263525);
  public static final Translation2d SWERVE_BACK_RIGHT_LOCATION =
      new Translation2d(-0.263525, -0.263525);

  public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.84);
  public static final double SWERVE_STEER_GEARING_REDUCTION = 7.0 / 150.0;

  public static final double SWERVE_DRIVE_GEARING_REDUCTION =
      50.0 * 16.0 * 45.0 / 14.0 / 28.0 / 15.0;
  public static final double SWERVE_STEER_KV = 0.0;
  public static final double SWERVE_STEER_KP = 20;
  public static final double SWERVE_STEER_KI = 0.0;
  public static final double SWERVE_STEER_KD = 0.0;
  public static final double SWERVE_STEER_KS = 0.0;

  public static final int SWERVE_DRIVE_VOLTAGE_PEAK_FORWARD_VOLTAGE = 12;
  public static final int SWERVE_DRIVE_VOLTAGE_PEAK_REVERSE_VOLTAGE = -12;
  public static final double SWERVE_DRIVE_CURRENT_LIMIT = 35.0;
  public static final boolean SWERVE_DRIVE_LIMITS_ENABLE = true;

  public static final double SWERVE_DRIVE_KP = 0.24;
  public static final double SWERVE_DRIVE_KI = 0.0;
  public static final double SWERVE_DRIVE_KD = 0.0;
  public static final double SWERVE_DRIVE_KV = 0.1185;
  public static final double SWERVE_DRIVE_KS = 0.0;

  public static final double STEER_MOTOR_LIMITS = 35;
  public static final boolean SWERVE_MOTOR_LIMITS_ENABLED = true;
  public static final boolean SWERVE_USE_FOC = true;
  // These PID values for translation and rotation (not snaps) are insane and almost seem like they
  // don't actually influence the path following.
  // When you set PathPlanner to run at 3m/s and 3m/s/s it commands an insane speed, which can't be
  // followed.
  // The result is that the robot is Fast Enough.
  // We don't have enough time to debug the issue more.
  public static final PIDConstants SWERVE_TRANSLATION_PID = new PIDConstants(12.5, 0, 0);
  public static final PIDConstants SWERVE_ROTATION_PID = new PIDConstants(10, 0, 0.1);
  public static final PIDConstants SWERVE_ROTATION_SNAP_PID = new PIDConstants(3, 0, 0);
  public static final boolean SWERVE_TRANSLATION_PID_INVERT = true;
  public static final boolean SWERVE_ROTATION_PID_INVERT = false;
  public static final boolean SWERVE_ROTATION_SNAP_PID_INVERT = true;

  public static final int SWERVE_FL_DRIVE_MOTOR_ID = 2;
  public static final int SWERVE_FL_STEER_MOTOR_ID = 3;
  public static final int SWERVE_FL_CANCODER_ID = 10;
  public static final SwerveModuleConstants SWERVE_FL_CONSTANTS =
      new SwerveModuleConstants(
          Rotation2d.fromDegrees(-177.45).unaryMinus(), SwerveCorner.FRONT_LEFT, true, true);

  public static final int SWERVE_FR_DRIVE_MOTOR_ID = 4;
  public static final int SWERVE_FR_STEER_MOTOR_ID = 5;
  public static final int SWERVE_FR_CANCODER_ID = 11;
  public static final SwerveModuleConstants SWERVE_FR_CONSTANTS =
      new SwerveModuleConstants(
          Rotation2d.fromDegrees(-46.49).unaryMinus(), SwerveCorner.FRONT_RIGHT, true, true);

  public static final int SWERVE_BL_DRIVE_MOTOR_ID = 6;
  public static final int SWERVE_BL_STEER_MOTOR_ID = 7;
  public static final int SWERVE_BL_CANCODER_ID = 12;
  public static final SwerveModuleConstants SWERVE_BL_CONSTANTS =
      new SwerveModuleConstants(
          Rotation2d.fromDegrees(-64.77).unaryMinus(), SwerveCorner.BACK_LEFT, true, true);

  public static final int SWERVE_BR_DRIVE_MOTOR_ID = 8;
  public static final int SWERVE_BR_STEER_MOTOR_ID = 9;
  public static final int SWERVE_BR_CANCODER_ID = 13;
  public static final SwerveModuleConstants SWERVE_BR_CONSTANTS =
      new SwerveModuleConstants(
          Rotation2d.fromDegrees(31.03).unaryMinus(), SwerveCorner.BACK_RIGHT, true, true);

  private Config() {}
}
