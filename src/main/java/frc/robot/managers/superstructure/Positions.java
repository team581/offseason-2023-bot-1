// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;

public class Positions {
  public static final SuperstructurePosition STOWED =
      new SuperstructurePosition(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0));

  public static final SuperstructurePosition INTAKING_CUBE_FLOOR =
      new SuperstructurePosition(Rotation2d.fromDegrees(20), Rotation2d.fromDegrees(135));
  public static final SuperstructurePosition INTAKING_CUBE_SHELF =
      new SuperstructurePosition(Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(180));

  public static final SuperstructurePosition CUBE_NODE_LOW_FRONT =
      new SuperstructurePosition(Rotation2d.fromDegrees(-20), Rotation2d.fromDegrees(-135));
  public static final SuperstructurePosition CUBE_NODE_LOW_BACK =
      new SuperstructurePosition(Rotation2d.fromDegrees(20), Rotation2d.fromDegrees(135));
  public static final SuperstructurePosition CUBE_NODE_MID =
      new SuperstructurePosition(Rotation2d.fromDegrees(-75), Rotation2d.fromDegrees(-190));
  public static final SuperstructurePosition CUBE_NODE_HIGH =
      new SuperstructurePosition(Rotation2d.fromDegrees(-100), Rotation2d.fromDegrees(-190));

  public static final SuperstructurePosition INTAKING_CONE_FLOOR =
      new SuperstructurePosition(Rotation2d.fromDegrees(20), Rotation2d.fromDegrees(135));
  public static final SuperstructurePosition INTAKING_CONE_SHELF =
      new SuperstructurePosition(Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(180));
  public static final SuperstructurePosition INTAKING_CONE_SINGLE_SUBSTATION =
      new SuperstructurePosition(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(30));

  public static final SuperstructurePosition CONE_NODE_LOW_FRONT =
      new SuperstructurePosition(Rotation2d.fromDegrees(-20), Rotation2d.fromDegrees(-135));
  public static final SuperstructurePosition CONE_NODE_LOW_BACK =
      new SuperstructurePosition(Rotation2d.fromDegrees(20), Rotation2d.fromDegrees(135));
  public static final SuperstructurePosition CONE_NODE_MID =
      new SuperstructurePosition(Rotation2d.fromDegrees(-75), Rotation2d.fromDegrees(-190));
  public static final SuperstructurePosition CONE_NODE_HIGH =
      new SuperstructurePosition(Rotation2d.fromDegrees(-100), Rotation2d.fromDegrees(-190));

  private Positions() {}
}
