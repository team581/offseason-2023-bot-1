package frc.robot.managers.superstructure;

import frc.robot.util.scheduling.LifecycleSubsystem;

public class SuperstructureManager extends LifecycleSubsystem {
  private ShoulderSubsystem shoulder;
  private WristSubsystem wrist;
  private IntakeSubsystem intake;
  private SuperstructurePosition goalPosition;

  public SuperstructureManager (ShoulderSubsystem shoulder, WristSubsystem wrist, IntakeSubsystem intake) {
    this.shoulder = shoulder;
    this.wrist = wrist;
    this.intake = intake;
  }

// create a setGoal method - you give it a position, it returns void
private void setGoal(SuperstructurePosition goalPosition) {
  this.goalPosition = goalPosition;
}

// create an atGoal() method


// in enabledperiodic, go to the goal position

  // create a setPositionCommand method, which finishes once at goal
}
