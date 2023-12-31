// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends LifecycleSubsystem {
  private final CANSparkMax motor;
  private RelativeEncoder encoder;
  private SparkMaxPIDController pid;
  private Rotation2d goalAngle = new Rotation2d();

  public WristSubsystem(CANSparkMax motor) {
    super(SubsystemPriority.WRIST);

    this.motor = motor;
    encoder = motor.getEncoder();
    pid = motor.getPIDController();
    motor.setSmartCurrentLimit(25);
    pid.setP(5.0);
    pid.setI(0.0);
    pid.setD(0.0);
    pid.setFF(0.0);

    encoder.setPosition(Rotation2d.fromDegrees(-74.0).getRotations());
    encoder.setPositionConversionFactor(1.0 / 40.9);

    motor.burnFlash();
  }

  @Override
  public void enabledPeriodic() {
    pid.setReference(goalAngle.getRotations(), ControlType.kPosition);
  }

  @Override
  public void robotPeriodic() {
    Logger.getInstance().recordOutput("Wrist/Velocity", encoder.getVelocity());
    Logger.getInstance().recordOutput("Wrist/Angle", getWristAngle().getDegrees());
    Logger.getInstance().recordOutput("Wrist/GoalAngle", goalAngle.getDegrees());
    Logger.getInstance().recordOutput("Wrist/DutyCycleOutput", motor.getAppliedOutput());
    Logger.getInstance().recordOutput("Wrist/StatorCurrent", motor.getOutputCurrent());
  }

  public void set(Rotation2d angle) {
    goalAngle = angle;
  }

  public boolean atAngle(Rotation2d angle) {
    double actualAngle = getWristAngle().getDegrees();
    return Math.abs(actualAngle - angle.getDegrees()) < 5;
  }

  private Rotation2d getWristAngle() {
    return Rotation2d.fromRotations(encoder.getPosition());
  }
}
