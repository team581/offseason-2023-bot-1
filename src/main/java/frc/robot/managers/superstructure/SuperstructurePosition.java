// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;

public class SuperstructurePosition {
  public final Rotation2d shoulderAngle;
  public final Rotation2d wristAngle;

  public SuperstructurePosition(Rotation2d shoulderAngle, Rotation2d wristAngle) {
    this.shoulderAngle = shoulderAngle;
    this.wristAngle = wristAngle;
  }

  @Override
  public boolean equals(Object obj) {
    if (obj instanceof SuperstructurePosition) {
      SuperstructurePosition position = (SuperstructurePosition) obj;

      return shoulderAngle.equals(position.shoulderAngle) && wristAngle.equals(position.wristAngle);
    }

    return false;
  }
}
