// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers.superstructure;

import frc.robot.intake.IntakeState;

public class States {
  public static final SuperstructureState STOWED =
      new SuperstructureState(Positions.STOWED, IntakeState.STOPPED, true);

  public static final SuperstructureState INTAKING_CUBE_FLOOR =
      new SuperstructureState(Positions.INTAKING_CUBE_FLOOR, IntakeState.INTAKE_CUBE, true);
  public static final SuperstructureState INTAKING_CUBE_SHELF =
      new SuperstructureState(Positions.INTAKING_CUBE_SHELF, IntakeState.INTAKE_CUBE, true);

  public static final SuperstructureScoringState CUBE_NODE_LOW_BACK =
      new SuperstructureScoringState(Positions.CUBE_NODE_LOW_BACK, IntakeState.OUTTAKE_CUBE);
      public static final SuperstructureScoringState CUBE_NODE_LOW_FRONT =
      new SuperstructureScoringState(Positions.CUBE_NODE_LOW_FRONT, IntakeState.OUTTAKE_CUBE);
  public static final SuperstructureScoringState CUBE_NODE_MID =
      new SuperstructureScoringState(Positions.CUBE_NODE_MID, IntakeState.OUTTAKE_CUBE);
  public static final SuperstructureScoringState CUBE_NODE_HIGH =
      new SuperstructureScoringState(Positions.CUBE_NODE_HIGH, IntakeState.OUTTAKE_CUBE);

  public static final SuperstructureState INTAKING_CONE_FLOOR =
      new SuperstructureState(Positions.INTAKING_CONE_FLOOR, IntakeState.INTAKE_CONE, true);
  public static final SuperstructureState INTAKING_CONE_SHELF =
      new SuperstructureState(Positions.INTAKING_CONE_SHELF, IntakeState.INTAKE_CONE, true);
  public static final SuperstructureState INTAKING_CONE_SINGLE_SUBSTATION =
      new SuperstructureState(
          Positions.INTAKING_CONE_SINGLE_SUBSTATION, IntakeState.INTAKE_CONE, true);

  public static final SuperstructureScoringState CONE_NODE_LOW_BACK =
      new SuperstructureScoringState(Positions.CONE_NODE_LOW_BACK, IntakeState.OUTTAKE_CONE);
    public static final SuperstructureScoringState CONE_NODE_LOW_FRONT =
      new SuperstructureScoringState(Positions.CONE_NODE_LOW_FRONT, IntakeState.OUTTAKE_CONE);
  public static final SuperstructureScoringState CONE_NODE_MID =
      new SuperstructureScoringState(Positions.CONE_NODE_MID, IntakeState.OUTTAKE_CONE);
  public static final SuperstructureScoringState CONE_NODE_HIGH =
      new SuperstructureScoringState(Positions.CONE_NODE_HIGH, IntakeState.OUTTAKE_CONE);

  public static final SuperstructureState YEET_CONE =
      new SuperstructureState(Positions.YEET_CONE, IntakeState.OUTTAKE_CONE, true);

  // Template to save time: public static final SuperstructureState  = new
  // SuperstructureState(positions.);

  private States() {}
}
