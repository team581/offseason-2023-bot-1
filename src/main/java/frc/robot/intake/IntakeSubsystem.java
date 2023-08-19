// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.revrobotics.*;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

/** Add your docs here. */
public class IntakeSubsystem extends LifecycleSubsystem {
  private CANSparkMax motor;
  public IntakeState state;
  public IntakeSubsystem(CANSparkMax motor) {
    super(SubsystemPriority.INTAKE);
    this.motor = motor;
  }
  public void setState(IntakeState intakeState){
    state = intakeState;
  }

  @Override
  public void enabledPeriodic() {

  }
}
