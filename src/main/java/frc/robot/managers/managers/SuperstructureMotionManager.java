// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers.managers;

import frc.robot.shoulder.ShoulderSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.wrist.WristSubsystem;
import java.util.ArrayList;

public class SuperstructureMotionManager extends LifecycleSubsystem {
  public final ShoulderSubsystem shoulder;
  public final WristSubsystem wrist;
  private SuperstructurePosition currentPoint = Positions.STOWED;
  private SuperstructurePosition goalPosition = Positions.STOWED;
  private final ArrayList<SuperstructurePosition> positionList =
      new ArrayList<SuperstructurePosition>();

  public SuperstructureMotionManager(ShoulderSubsystem shoulder, WristSubsystem wrist) {
    super(SubsystemPriority.SUPERSTRUCTURE_MOTION_MANAGER);

    this.shoulder = shoulder;
    this.wrist = wrist;
  }

  public boolean atPosition(SuperstructurePosition position) {
    return shoulder.atAngle(position.shoulderAngle) && wrist.atAngle(position.wristAngle);
  }

  public void set(SuperstructurePosition newGoalPosition) {
    if (!newGoalPosition.equals(goalPosition)) {
      positionList.clear();
      positionList.add(
          new SuperstructurePosition(currentPoint.shoulderAngle, Positions.STOWED.wristAngle));
      positionList.add(
          new SuperstructurePosition(newGoalPosition.shoulderAngle, Positions.STOWED.wristAngle));
      positionList.add(newGoalPosition);

      goalPosition = newGoalPosition;
    }
  }

  @Override
  public void enabledInit() {
    positionList.clear();
  }

  @Override
  public void enabledPeriodic() {
    if (atPosition(currentPoint) && !positionList.isEmpty()) {
      currentPoint = positionList.remove(0);
    }

    shoulder.set(currentPoint.shoulderAngle);
    wrist.set(currentPoint.wristAngle);
  }
}
