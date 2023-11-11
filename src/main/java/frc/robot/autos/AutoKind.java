// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;

public enum AutoKind {
  DO_NOTHING("", 0.01, 0.01, false),

  BLUE_FLAT_SIDE_3("BlueShortSide3", 3, 3, false),
  BLUE_BUMP_SIDE_3("BlueLongSide3", 3, 3, false),
  RED_FLAT_SIDE_3("RedShortSide3", 3, 3, false),
  RED_BUMP_SIDE_3("RedLongSide3", 3, 3, false),
  TEST("Test", 2, 2, false),
  RED_TAXI_1("RedTaxi1", 2, 2, false),
  BLUE_TAXI_1("BlueTaxi1", 2, 2, false),
  BLUE_MID_1_BALANCE("BlueMid1Balance", 1.5, 2, true),
  RED_MID_1_BALANCE("RedMid1Balance", 1.5, 2, true);
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
