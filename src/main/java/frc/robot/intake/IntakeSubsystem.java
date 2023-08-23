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
  private IntakeState state;
  private HeldGamePiece gamePiece = HeldGamePiece.NOTHING;

  public IntakeSubsystem(CANSparkMax motor) {
    super(SubsystemPriority.INTAKE);
    this.motor = motor;
  }

  public void setState(IntakeState intakeState) {
    state = intakeState;
  }

  @Override
  public void enabledPeriodic() {
    if (state == IntakeState.INTAKE_CUBE) {
      motor.set(0.5);
      gamePiece = HeldGamePiece.CUBE;
    } else if (state == IntakeState.INTAKE_CONE) {
      motor.set(-1);
      gamePiece = HeldGamePiece.CONE;
    } else if (state == IntakeState.OUTTAKE_CONE) {
      motor.set(0.4);
      gamePiece = HeldGamePiece.NOTHING;
    } else if (state == IntakeState.OUTTAKE_CUBE) {
      motor.set(-0.3);
      gamePiece = HeldGamePiece.NOTHING;
    } else {
      motor.set(0.0);
    }
  }

  public HeldGamePiece getGamePiece() {
    return gamePiece;
  }

  public void setGamePiece(HeldGamePiece gamePiece) {
    this.gamePiece = gamePiece;
  }
}
