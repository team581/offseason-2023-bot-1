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
  private SuperstructureState goalState;

  public SuperstructureManager(SuperstructureMotionManager motionManager, IntakeSubsystem intake) {
    super(SubsystemPriority.SUPERSTRUCTURE_MANAGER);

    this.motionManager = motionManager;
    this.shoulder = motionManager.shoulder;
    this.wrist = motionManager.wrist;
    this.intake = intake;
  }

  private void setGoal(SuperstructureState goalState) {
    this.goalState = goalState;
  }

  public boolean atGoal(SuperstructureState state) {
    return shoulder.atAngle(state.position.shoulderAngle.getDegrees())
        && wrist.atAngle(state.position.wristAngle.getDegrees());
  }

  // in enabledperiodic, go to the goal position
  @Override
  public void enabledPeriodic() {
    motionManager.set(goalState.position);
  }

  public Command setStateCommand(SuperstructureState newGoalState) {
    return Commands.runOnce(() -> setGoal(newGoalState))
        .andThen(Commands.waitUntil(() -> atGoal(newGoalState)));
  }
}
