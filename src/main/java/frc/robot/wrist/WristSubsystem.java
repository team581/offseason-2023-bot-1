// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class WristSubsystem extends LifecycleSubsystem {
  private final CANSparkMax motor;
  private RelativeEncoder encoder;
  private SparkMaxPIDController pid;
  private double goalAngle = 0.0;

  public WristSubsystem(CANSparkMax motor) {
    super(SubsystemPriority.WRIST);

    this.motor = motor;
    encoder = motor.getEncoder();
    pid = motor.getPIDController();

    pid.setP(5);
    pid.setI(0);
    pid.setD(0);

    encoder.setPosition(0);
    encoder.setPositionConversionFactor(50);
  }

  @Override
  public void enabledPeriodic() {
    pid.setReference(goalAngle / 360.0, ControlType.kSmartMotion);
  }

  public double set(double angle) {
    goalAngle = angle;
    return goalAngle;
  }

// create a method set(double angle), which sets the goal angle
  // update setPositionCommand to use the set() method

  public Command setPositionCommand(double angle) {
    return run(() -> {
          set(angle);
        })
        .until(() -> atAngle(angle));
  }

  private boolean atAngle(double angle) {
    return Math.abs(getWristAngle() - angle) < 1;
  }

  private double getWristAngle() {
    return encoder.getPosition() * 360.0;
  }
}
