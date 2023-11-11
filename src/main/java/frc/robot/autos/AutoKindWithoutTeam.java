// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

public enum AutoKindWithoutTeam {
  DO_NOTHING(AutoKind.DO_NOTHING, AutoKind.DO_NOTHING),
  BUMP_SIDE_3(AutoKind.RED_BUMP_SIDE_3, AutoKind.BLUE_BUMP_SIDE_3),
  // FLAT_SIDE_3(AutoKind.RED_FLAT_SIDE_3, AutoKind.BLUE_FLAT_SIDE_3),
  // TEST(AutoKind.TEST, AutoKind.TEST),
  TAXI_1(AutoKind.RED_TAXI_1, AutoKind.BLUE_TAXI_1),
  MID_1_BALANCE(AutoKind.RED_MID_1_BALANCE, AutoKind.BLUE_MID_1_BALANCE),
  BUMP_SIDE_2(AutoKind.RED_BUMP_SIDE_2, AutoKind.BLUE_BUMP_SIDE_2);

  public final AutoKind redVersion;
  public final AutoKind blueVersion;

  private AutoKindWithoutTeam(AutoKind redVersion, AutoKind blueVersion) {
    this.redVersion = redVersion;
    this.blueVersion = blueVersion;
  }
}
