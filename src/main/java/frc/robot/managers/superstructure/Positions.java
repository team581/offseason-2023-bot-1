// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;

public class Positions {
  public static final SuperstructurePosition STOWED =
      new SuperstructurePosition(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0));

  public static final SuperstructurePosition INTAKING_CUBE_FLOOR =
      new SuperstructurePosition(Rotation2d.fromDegrees(-31), Rotation2d.fromDegrees(-140));
  public static final SuperstructurePosition INTAKING_CUBE_SHELF =
      new SuperstructurePosition(Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(180));

  public static final SuperstructurePosition CUBE_NODE_LOW_FRONT =
      new SuperstructurePosition(Rotation2d.fromDegrees(-24.2), Rotation2d.fromDegrees(-133.5));
  public static final SuperstructurePosition CUBE_NODE_LOW_BACK =
      new SuperstructurePosition(Rotation2d.fromDegrees(4.7), Rotation2d.fromDegrees(93.9));
  public static final SuperstructurePosition CUBE_NODE_MID =
      new SuperstructurePosition(Rotation2d.fromDegrees(28), Rotation2d.fromDegrees(68));
  public static final SuperstructurePosition CUBE_NODE_HIGH =
      new SuperstructurePosition(Rotation2d.fromDegrees(100), Rotation2d.fromDegrees(187));

  public static final SuperstructurePosition INTAKING_CONE_FLOOR =
      new SuperstructurePosition(Rotation2d.fromDegrees(-32), Rotation2d.fromDegrees(-152));
  public static final SuperstructurePosition INTAKING_CONE_SHELF =
      new SuperstructurePosition(Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(180));
  public static final SuperstructurePosition INTAKING_CONE_SINGLE_SUBSTATION =
      new SuperstructurePosition(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(42));

  public static final SuperstructurePosition CONE_NODE_LOW_FRONT =
      new SuperstructurePosition(Rotation2d.fromDegrees(-19), Rotation2d.fromDegrees(-118.6));
  public static final SuperstructurePosition CONE_NODE_LOW_BACK =
      new SuperstructurePosition(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(89));
  public static final SuperstructurePosition CONE_NODE_MID =
      new SuperstructurePosition(Rotation2d.fromDegrees(125), Rotation2d.fromDegrees(-90));
  public static final SuperstructurePosition CONE_NODE_HIGH =
      new SuperstructurePosition(Rotation2d.fromDegrees(130), Rotation2d.fromDegrees(-132));

  private Positions() {}
}
