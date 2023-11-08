// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;

public enum AutoKind {
  DO_NOTHING("", 0.01, 0.01, false),

  BLUE_FLAT_SIDE_3("BlueShortSide3", 4, 4, false),
  BLUE_BUMP_SIDE_3("BlueLongSide3", 4, 4, false),
  RED_FLAT_SIDE_3("RedShortSide3", 4, 4, false),
  RED_BUMP_SIDE_3("RedLongSide3", 4, 4, false),
  TEST("test", 2, 2, false);

  public final String pathName;
  public final PathConstraints constraints;
  public final boolean autoBalance;

  private AutoKind(
      String pathName, double maxVelocity, double maxAcceleration, boolean autoBalance) {
    this.pathName = pathName;
    this.constraints = new PathConstraints(maxVelocity, maxAcceleration);
    this.autoBalance = autoBalance;
  }
}
