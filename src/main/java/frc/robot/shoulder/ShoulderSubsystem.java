// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shoulder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

public class ShoulderSubsystem extends LifecycleSubsystem {
  private final CANSparkMax motor;
  private final CANSparkMax motorFollower;

  private RelativeEncoder encoder;
  private SparkMaxPIDController pid;
  private Rotation2d goalAngle = new Rotation2d();

  public ShoulderSubsystem(CANSparkMax motor, CANSparkMax followerMotor) {
    super(SubsystemPriority.SHOULDER);
    this.motor = motor;
    this.motorFollower = followerMotor;
    followerMotor.follow(motor);
    encoder = motor.getEncoder();
    pid = motor.getPIDController();
    motor.setSmartCurrentLimit(35);
    followerMotor.setSmartCurrentLimit(35);

    pid.setP(0);
    pid.setI(0);
    pid.setD(0);
    pid.setFF(0);

    encoder.setPosition(0);
    encoder.setPositionConversionFactor(50);
  }

  @Override
  public void enabledPeriodic() {
    pid.setReference(goalAngle.getRotations(), ControlType.kSmartMotion);
  }

  @Override
  public void robotPeriodic() {
    Logger.getInstance().recordOutput("Shoulder/MainMotor/Velocity", encoder.getVelocity());
    Logger.getInstance().recordOutput("Shoulder/FollowerMotor/Velocity", encoder.getVelocity());
    Logger.getInstance().recordOutput("Shoulder/MainMotor/Angle", getWristAngle().getDegrees());
    Logger.getInstance().recordOutput("Shoulder/FollowerMotor/Angle", getWristAngle().getDegrees());
    Logger.getInstance().recordOutput("Shoulder/MainMotor/GoalAngle", goalAngle.getDegrees());
    Logger.getInstance().recordOutput("Shoulder/FollowerMotor/GoalAngle", goalAngle.getDegrees());
    Logger.getInstance()
        .recordOutput("Shoulder/MainMotor/DutyCycleOutput", motor.getAppliedOutput());
    Logger.getInstance()
        .recordOutput("Shoulder/FollowerMotor/DutyCycleOutput", motor.getAppliedOutput());
    Logger.getInstance().recordOutput("Shoulder/MainMotor/StatorCurrent", motor.getOutputCurrent());
    Logger.getInstance()
        .recordOutput("Shoulder/FollowerMotor/StatorCurrent", motor.getOutputCurrent());
  }

  public void set(Rotation2d angle) {
    goalAngle = angle;
  }

  public boolean atAngle(Rotation2d angle) {
    double actualAngle = getWristAngle().getDegrees();
    return Math.abs(actualAngle - angle.getDegrees()) < 1;
  }

  private Rotation2d getWristAngle() {
    return Rotation2d.fromRotations(encoder.getPosition());
  }
}
