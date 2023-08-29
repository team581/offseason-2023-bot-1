// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.revrobotics.*;
import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class IntakeSubsystem extends LifecycleSubsystem {
  private final LinearFilter voltageFilter = LinearFilter.movingAverage(7);
  private final LinearFilter velocityFilter = LinearFilter.movingAverage(7);
  // 7 is a placeholder not tuned
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;
  private IntakeState goalState;
  private HeldGamePiece gamePiece = HeldGamePiece.NOTHING;

  public IntakeSubsystem(CANSparkMax motor) {
    super(SubsystemPriority.INTAKE);
    this.motor = motor;
    encoder = motor.getEncoder();
  }

  public void setGoalState(IntakeState intakeState) {
    if (goalState != intakeState) {
      if (goalState == IntakeState.INTAKE_CONE || goalState == IntakeState.INTAKE_CUBE) {
        gamePiece = HeldGamePiece.NOTHING;
      }
    }

    goalState = intakeState;
  }

  @Override
  public void robotPeriodic() {
    Logger.getInstance().recordOutput("Intake/State", goalState.toString());
    Logger.getInstance().recordOutput("Intake/HeldGamePiece", gamePiece.toString());
    Logger.getInstance().recordOutput("Intake/Current", motor.getOutputCurrent());
    Logger.getInstance().recordOutput("Intake/DutyCycleOutput", motor.getAppliedOutput());
    Logger.getInstance().recordOutput("Intake/Velocity", motor.getEncoder().getVelocity());
  }

  @Override
  public void enabledPeriodic() {
    if (goalState == IntakeState.OUTTAKE_CONE) {
      motor.set(0.4);
    } else if (goalState == IntakeState.OUTTAKE_CUBE) {
      motor.set(-0.3);
    } else if (gamePiece == HeldGamePiece.CUBE) {
      motor.set(0.15);
    } else if (gamePiece == HeldGamePiece.CONE) {
      motor.set(-0.1);
    } else if (goalState == IntakeState.INTAKE_CUBE) {
      motor.set(0.5);
    } else if (goalState == IntakeState.INTAKE_CONE) {
      motor.set(-1);
    } else {
      motor.disable();
    }

    // Game piece detection
    double motorVelocity = velocityFilter.calculate(encoder.getVelocity());
    double intakeVoltage = voltageFilter.calculate(motor.getAppliedOutput()) * 12.0;
    double theoreticalSpeed = intakeVoltage * (5700.0 / 12.0); // Neo Max is 5700
    double threshold = theoreticalSpeed * 0.5;
    if (motorVelocity < threshold && goalState == IntakeState.INTAKE_CONE) {
      gamePiece = HeldGamePiece.CONE;
    } else if (motorVelocity < threshold && goalState == IntakeState.INTAKE_CUBE) {
      gamePiece = HeldGamePiece.CUBE;
    } else if (motorVelocity > threshold
        && (goalState == IntakeState.OUTTAKE_CONE || goalState == IntakeState.OUTTAKE_CUBE)) {
      gamePiece = HeldGamePiece.NOTHING;
    }
  }

  public IntakeState getIntakeState() {
    return goalState;
  }

  public boolean atGoal(IntakeState state) {
    if (goalState != state) {
      return false;
    }
    if (state == IntakeState.OUTTAKE_CONE || state == IntakeState.OUTTAKE_CUBE) {
      return gamePiece == HeldGamePiece.NOTHING;
    }
    if (state == IntakeState.STOPPED) {
      return true;
    }
    if (state == IntakeState.INTAKE_CUBE) {
      return gamePiece == HeldGamePiece.CUBE;
    }
    if (state == IntakeState.INTAKE_CONE) {
      return gamePiece == HeldGamePiece.CONE;
    }
    return false;
  }

  public HeldGamePiece getGamePiece() {
    return gamePiece;
  }

  public void setGamePiece(HeldGamePiece gamePiece) {
    this.gamePiece = gamePiece;
  }
}
