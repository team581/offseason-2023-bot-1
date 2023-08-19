// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.managers;

import edu.wpi.first.math.geometry.Rotation2d;

public class SuperstructurePosition {
  private Rotation2d shoulderAngle;
  private Rotation2d wristAngle;

  public SuperstructurePosition(Rotation2d shoulderAngle, Rotation2d wristAngle) {
    this.shoulderAngle = shoulderAngle;
    this.wristAngle = wristAngle;
  }
}

