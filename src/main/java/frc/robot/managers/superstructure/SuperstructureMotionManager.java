package frc.robot.managers.superstructure;

import frc.robot.util.scheduling.LifecycleSubsystem;

public class SuperstructureMotionManager extends LifecycleSubsystem {
  private ShoulderSubsystem shoulder;
  private WristSubsystem wrist;
  private SuperstructurePosition currentPoint = Positions.STOWED;
  private SuperstructurePosition goalPosition = Positions.STOWED;

  public SuperstructureMotionManager (ShoulderSubsystem shoulder, WristSubsystem wrist) {
    this.shoulder = shoulder;
    this.wrist = wrist;
  }

  public void set(SuperstructurePosition position) {
  }

  @Override
  public void enabledperiodic() {
      shoulder.setAngle(currentpoint.shoulderAngle);
      wrist.setAngle(currentpoint.wristAngle);
  }
}
