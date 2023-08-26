// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.wrist.WristSubsystem;

public class SuperstructureManager extends LifecycleSubsystem {
  private SuperstructureMotionManager motionManager;
  private WristSubsystem shoulder;
  private WristSubsystem wrist;
  private IntakeSubsystem intake;
  private SuperstructurePosition goalPosition;

  public SuperstructureManager(SuperstructureMotionManager motionManager, IntakeSubsystem intake) {
    super(SubsystemPriority.SUPERSTRUCTURE_MANAGER);

    this.motionManager = motionManager;
    this.shoulder = motionManager.shoulder;
    this.wrist = motionManager.wrist;
    this.intake = intake;
  }

  private void setGoal(SuperstructurePosition goalPosition) {
    this.goalPosition = goalPosition;
  }

  public boolean atPosition(SuperstructurePosition position) {
    return shoulder.atAngle(position.shoulderAngle.getDegrees())
        && wrist.atAngle(position.wristAngle.getDegrees());
  }

  // in enabledperiodic, go to the goal position
  @Override
  public void enabledPeriodic() {
    motionManager.set(goalPosition);
  }

  // create a setPositionCommand method, which finishes once at goal
  public Command setGoalCommand(SuperstructurePosition newGoalPosition) {
    return Commands.runOnce(() -> setGoal(newGoalPosition))
        .andThen(Commands.waitUntil(() -> atPosition(newGoalPosition)));
  }
}
