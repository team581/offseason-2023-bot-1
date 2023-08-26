// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.revrobotics.*;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

/** Add your docs here. */
public class IntakeSubsystem extends LifecycleSubsystem {
  private CANSparkMax motor;
  private IntakeState goalState;
  private HeldGamePiece gamePiece = HeldGamePiece.NOTHING;

  public IntakeSubsystem(CANSparkMax motor) {
    super(SubsystemPriority.INTAKE);
    this.motor = motor;
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
  public void enabledPeriodic() {
    if (goalState == IntakeState.OUTTAKE_CONE) {
      motor.set(0.4);
      // gamePiece = HeldGamePiece.NOTHING;
    } else if (goalState == IntakeState.OUTTAKE_CUBE) {
      motor.set(-0.3);
      // gamePiece = HeldGamePiece.NOTHING;
    } else if (gamePiece == HeldGamePiece.CUBE) {
      motor.set(0.15);
    } else if (gamePiece == HeldGamePiece.CONE) {
      motor.set(-0.1);
    } else if (goalState == IntakeState.INTAKE_CUBE) {
      motor.set(0.5);
      // gamePiece = HeldGamePiece.CUBE;
    } else if (goalState == IntakeState.INTAKE_CONE) {
      motor.set(-1);
      // gamePiece = HeldGamePiece.CONE;
    } else {
      motor.disable();
    }
  }

  // create an atGoal(IntakeMode mode) method
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
