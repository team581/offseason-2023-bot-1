// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers.superstructure;

import frc.robot.intake.IntakeState;

public class SuperstructureScoringState {
  public final SuperstructureState aligning;
  public final SuperstructureState scoring;

  public SuperstructureScoringState(
      SuperstructurePosition position, IntakeState scoringIntakeState) {
    this.aligning = new SuperstructureState(position, IntakeState.STOPPED, true);
    this.scoring = new SuperstructureState(position, scoringIntakeState);
  }
}
