package frc.robot.managers.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;

public class SuperstructureManager extends LifecycleSubsystem {
  private SuperstructureMotionManager motionManager;
  private ShoulderSubsystem shoulder;
  private WristSubsystem wrist;
  private IntakeSubsystem intake;
  private SuperstructurePosition goalPosition;

  public SuperstructureManager (SuperstructureMotionManager motionManager, ShoulderSubsystem shoulder, WristSubsystem wrist, IntakeSubsystem intake) {
    this.motionManager = motionManager;
    this.shoulder = shoulder;
    this.wrist = wrist;
    this.intake = intake;
  }

// create a setGoal method - you give it a position, it returns void
private void setGoal(SuperstructurePosition goalPosition) {
  this.goalPosition = goalPosition;
}

// create an atGoal() method
public boolean atGoal(Rotation2d shoulderAngle, Rotation2d wristAngle) {
  return shoulderAngle == goalPosition.shoulderAngle && wristAngle == goalPosition.wristAngle;
}

// in enabledperiodic, go to the goal position
@Override
public void enabledPeriodic() {
  motionmanager.set
}
  // create a setPositionCommand method, which finishes once at goal
}
