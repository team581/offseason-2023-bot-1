// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.config.Config;
import frc.robot.util.CircleConverter;
import frc.robot.util.CtreModuleState;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
  private static final CircleConverter DRIVE_MOTOR_WHEEL_CONVERTER =
      CircleConverter.fromDiameter(Config.WHEEL_DIAMETER);

  private final SwerveModuleConstants constants;
  private final TalonFX driveMotor;
  private final CANSparkMax steerMotor;
  private final DutyCycleOut driveVoltageOpenLoopRequest =
      new DutyCycleOut(0, Config.SWERVE_USE_FOC, true);
  private final VelocityVoltage driveVoltageClosedLoopRequest =
      new VelocityVoltage(0, Config.SWERVE_USE_FOC, 0, 0, false);
  private Rotation2d previousAngle = new Rotation2d();

  private StatusSignal<Double> driveMotorStatorCurrent;

  private final CANcoder cancoder;

  private final RelativeEncoder steerMotorEncoder;
  private final SparkMaxPIDController steerMotorPID;

  private boolean setAngle = false;

  public SwerveModule(
      SwerveModuleConstants constants,
      TalonFX driveMotor,
      CANSparkMax steerMotor,
      CANcoder cancoder) {
    this.constants = constants;
    this.driveMotor = driveMotor;
    this.steerMotor = steerMotor;
    this.cancoder = cancoder;

    steerMotorEncoder = steerMotor.getEncoder();
    steerMotorPID = steerMotor.getPIDController();

    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

    cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cancoderConfig.MagnetSensor.MagnetOffset = constants.angleOffset.getRotations();
    cancoder.getConfigurator().apply(cancoderConfig);

    TalonFXConfiguration driveMotorConfigs = new TalonFXConfiguration();

    driveMotorConfigs.Feedback.SensorToMechanismRatio = -1 * Config.SWERVE_DRIVE_GEARING_REDUCTION;

    driveMotorConfigs.Slot0.kP = Config.SWERVE_DRIVE_KP;
    driveMotorConfigs.Slot0.kI = Config.SWERVE_DRIVE_KI;
    driveMotorConfigs.Slot0.kD = Config.SWERVE_DRIVE_KD;
    driveMotorConfigs.Slot0.kV = Config.SWERVE_DRIVE_KV;
    driveMotorConfigs.Slot0.kS = Config.SWERVE_DRIVE_KS;

    driveMotorConfigs.Voltage.PeakForwardVoltage = 12;
    driveMotorConfigs.Voltage.PeakReverseVoltage = -12;

    driveMotorConfigs.CurrentLimits.SupplyCurrentLimit = 15;
    driveMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = false;

    if (constants.driveInversion) {
      driveMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    } else {
      driveMotorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }

    StatusCode driveStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      driveStatus = driveMotor.getConfigurator().apply(driveMotorConfigs);
      if (driveStatus.isOK()) break;
    }
    if (!driveStatus.isOK()) {
      System.out.println("Could not apply configs, error code: " + driveStatus.toString());
    }

    driveMotorStatorCurrent = driveMotor.getStatorCurrent();


    steerMotorPID.setP(Config.SWERVE_STEER_KP);
    steerMotorPID.setI(Config.SWERVE_STEER_KI);
    steerMotorPID.setD(Config.SWERVE_STEER_KD);
    steerMotorPID.setFF(Config.SWERVE_STEER_KV);
    steerMotorPID.setFeedbackDevice(steerMotorEncoder);
    steerMotor.setIdleMode(IdleMode.kBrake);

    steerMotorEncoder.setPositionConversionFactor(Config.SWERVE_STEER_GEARING_REDUCTION);
    steerMotorEncoder.setVelocityConversionFactor(Config.SWERVE_STEER_GEARING_REDUCTION);

    steerMotorPID.setPositionPIDWrappingEnabled(true);
    steerMotorPID.setPositionPIDWrappingMinInput(0.0);
    steerMotorPID.setPositionPIDWrappingMaxInput(1.0);

    steerMotor.setSmartCurrentLimit(15);

    steerMotor.setInverted(constants.angleInversion);

    steerMotor.burnFlash();
  }

  public void log() {
    Logger.getInstance()
        .recordOutput(
            "Swerve/" + constants.corner.toString() + "/DriveMotorPosition",
            getDriveMotorPosition());
    Logger.getInstance()
        .recordOutput(
            "Swerve/" + constants.corner.toString() + "/DriveMotorVelocity",
            getDriveMotorVelocity());

    Logger.getInstance()
        .recordOutput(
            "Swerve/" + constants.corner.toString() + "/DriveMotorStatorCurrent",
            driveMotorStatorCurrent.refresh().getValue());
    Logger.getInstance()
        .recordOutput(
            "Swerve/" + constants.corner.toString() + "/SteerMotorPosition",
            getSteerMotorPosition().getDegrees());
    Logger.getInstance()
        .recordOutput(
            "Swerve/" + constants.corner.toString() + "/CANCoderPositionWithOffset",
            getCancoderAngle().getDegrees());
    Logger.getInstance()
        .recordOutput(
            "Swerve/" + constants.corner.toString() + "/CANCoderPositionNoOffset",
            getCancoderAngle().minus(constants.angleOffset).getDegrees());
    Logger.getInstance()
        .recordOutput(
            "Swerve/" + constants.corner.toString() + "/EncoderConversionFactor",
            steerMotorEncoder.getPositionConversionFactor());
  }

  public void setDesiredState(
      SwerveModuleState state, boolean openLoop, boolean skipJitterOptimization) {
    final var steerMotorPosition = getSteerMotorPosition();
    state = CtreModuleState.optimize(state, steerMotorPosition);

    steerMotorPID.setReference(state.angle.getRotations(), CANSparkMax.ControlType.kPosition);

    boolean isStopped = Math.abs(state.speedMetersPerSecond) <= SwerveSubsystem.MAX_VELOCITY * 0.01;
    Rotation2d angle = isStopped && !skipJitterOptimization ? this.previousAngle : state.angle;
    this.previousAngle = angle;

    var wheelRotationsPerSecond =
        DRIVE_MOTOR_WHEEL_CONVERTER.distanceToRotations(state.speedMetersPerSecond);

    if (openLoop) {
      driveMotor.setControl(
          driveVoltageOpenLoopRequest.withOutput(
              state.speedMetersPerSecond / SwerveSubsystem.MAX_VELOCITY));
    } else {
      driveMotor.setControl(driveVoltageClosedLoopRequest.withVelocity(wheelRotationsPerSecond));
    }
  }

  public SwerveModuleState getState() {
    final var steerMotorPosition = getSteerMotorPosition();
    final var driveMotorVelocity = Units.inchesToMeters(getDriveMotorVelocity());

    return new SwerveModuleState(driveMotorVelocity, steerMotorPosition);
  }

  public SwerveModulePosition getPosition() {
    final var steerMotorPosition = getSteerMotorPosition();
    final var driveMotorPosition = getDriveMotorPosition();

    return new SwerveModulePosition(driveMotorPosition, steerMotorPosition);
  }

  private Rotation2d getCancoderAngle() {
    return Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValue());
  }

  public void resetSteerMotorAngle() {
    if (!setAngle) {
      REVLibError error = steerMotorEncoder.setPosition(getCancoderAngle().getRotations());
      Logger.getInstance()
          .recordOutput("Swerve/" + constants.corner.toString() + "/SetError", error.toString());
      if (error == REVLibError.kOk) {
        setAngle = true;
      }
    }
  }

  private Rotation2d getSteerMotorPosition() {
    double rotations = steerMotorEncoder.getPosition();
    return Rotation2d.fromRotations(rotations);
  }

  private double getDriveMotorPosition() {
    final var rotations = driveMotor.getPosition().getValue();
    final var meters = DRIVE_MOTOR_WHEEL_CONVERTER.rotationsToDistance(rotations);
    return meters;
  }

  private double getDriveMotorVelocity() {
    final var rotationsPerSecond = driveMotor.getVelocity().getValue();
    final var metersPerSecond = DRIVE_MOTOR_WHEEL_CONVERTER.rotationsToDistance(rotationsPerSecond);
    final var inchesPerSecond = metersPerSecond * 39.37;
    return inchesPerSecond;
  }
}
