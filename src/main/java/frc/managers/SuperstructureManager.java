package frc.managers;

public class SuperstructureManager {
  private ShoulderSubsystem shoulder;
  private WristSubsystem wrist;
  private IntakeSubsystem intake;

  public SuperstructureManager (ShoulderSubsystem shoulder, WristSubsystem wrist, IntakeSubsystem intake) {
    this.shoulder = shoulder;
    this.wrist = wrist;
    this.intake = intake;
  }

}
