package frc.robot.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.config.Config;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class SwerveSubsystem extends LifecycleSubsystem{

  public SwerveSubsystem(SubsystemPriority priority) {
    super(priority);

  }

  private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
   .withPigeon2Id(Config.PIGEON2_ID)
   .withSupportsPro(true)
   .withCANbusName(Config.CANIVORE_ID);
   private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
   .withDriveMotorGearRatio(kDriveGearRatio)
   .withSteerMotorGearRatio(kSteerGearRatio)
   .withWheelRadius(kWheelRadiusInches)
   .withSlipCurrent(800)
   .withSteerMotorGains(steerGains)
   .withDriveMotorGains(driveGains)
   .withSpeedAt12VoltsMps(6) // Theoretical free speed is 10 meters per second at 12v applied output
   .withSteerInertia(kSteerInertia)
   .withDriveInertia(kDriveInertia)
   .withFeedbackSource(SwerveModuleSteerFeedbackType.FusedCANcoder)
   .withCouplingGearRatio(kCoupleRatio) // Every 1 rotation of the azimuth results in couple ratio drive turns
   .withSteerMotorInverted(kSteerMotorReversed);
   private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
   Config.SWERVE_FL_STEER_MOTOR_ID, Config.SWERVE_FL_DRIVE_MOTOR_ID, Config.SWERVE_FL_CANCODER_ID, Rotation2d.fromDegrees(182.021484375).getRotations(), Config.SWERVE_FRONT_LEFT_LOCATION.getX(), Config.SWERVE_FRONT_LEFT_LOCATION.getY(), true);
private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
  Config.SWERVE_FR_STEER_MOTOR_ID, Config.SWERVE_FR_DRIVE_MOTOR_ID, Config.SWERVE_FR_CANCODER_ID, Rotation2d.fromDegrees(159.521484375).getRotations(), Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
  Config.SWERVE_BL_STEER_MOTOR_ID, Config.SWERVE_BR_DRIVE_MOTOR_ID, Config.SWERVE_BL_CANCODER_ID, Rotation2d.fromDegrees(2.548828125).getRotations(), Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
  Config.SWERVE_BR_STEER_MOTOR_ID, Config.SWERVE_BR_DRIVE_MOTOR_ID, Config.SWERVE_BR_CANCODER_ID, Rotation2d.fromDegrees(110.390625).getRotations(), Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);
}
