package frc.robot.wrist;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.wrist.WristSubsystem;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;

public class WristSubsystem extends LifecycleSubsystem {
  private CANSparkMax motor = new CANSparkMax(1, MotorType.kBrushless);
  private boolean zeroed = false;
  private boolean active = false;
  private RelativeEncoder encoder = motor.getEncoder();
  private double goalAngle = 0;

  public WristSubsystem(CANSparkMax motor) {
    super(SubsystemPriority.WRIST);

    this.motor = motor;
  }
  @Override
  public void enabledPeriodic() {
    if (active && zeroed) {
      encoder.setPosition(goalAngle / 360.0);
    } else {
      motor.disable();
    }
  }

  public Command setZero() {
    return runOnce(() -> {
      zeroed = true;
    });
  }
  public Command setDisabledCommand() {
    return runOnce(() -> {
      active = false;
    });
  }
  public Command setPositionCommand(double angle) {
    return run(() -> {
      goalAngle = angle;
      active = true;
    })
    .until(() -> atAngle(angle));
  }
  private boolean atAngle(double angle) {
    return Math.abs(getWristAngle() - angle) < 1;
  }
  private double getWristAngle() {
    return encoder.getPosition() * 360.0;
  }
}

