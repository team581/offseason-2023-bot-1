// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers.superstructure;

import frc.robot.intake.IntakeState;

public class SuperstructureState {
  public final SuperstructurePosition position;
  public final IntakeState intakeState;
  public final boolean intakeNow;

  public SuperstructureState(
      SuperstructurePosition position, IntakeState intakeMode, boolean intakeNow) {
    this.position = position;
    this.intakeState = intakeMode;
    this.intakeNow = intakeNow;
  }

  public SuperstructureState(SuperstructurePosition position, IntakeState intakeMode) {
    this(position, intakeMode, false);
  }

    @Override
  public boolean equals(Object obj) {
    if (obj instanceof SuperstructureState) {
      SuperstructureState state = (SuperstructureState) obj;

      return this == obj
          || (position.equals(state.position)
              && intakeState == state.intakeState
              && intakeNow == state.intakeNow);
    }

    return false;
  }
}
