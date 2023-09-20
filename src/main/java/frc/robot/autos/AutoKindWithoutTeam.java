// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

public enum AutoKindWithoutTeam {
  DO_NOTHING(AutoKind.DO_NOTHING, AutoKind.DO_NOTHING),
  LONG_SIDE_3(AutoKind.RED_LONG_SIDE_3, AutoKind.BLUE_LONG_SIDE_3),
  SHORT_SIDE_3(AutoKind.RED_SHORT_SIDE_3, AutoKind.BLUE_SHORT_SIDE_3);

  public final AutoKind redVersion;
  public final AutoKind blueVersion;

  private AutoKindWithoutTeam(AutoKind redVersion, AutoKind blueVersion) {
    this.redVersion = redVersion;
    this.blueVersion = blueVersion;
  }
}
