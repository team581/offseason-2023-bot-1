package frc.robot.wrist;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class WristSubsystem extends LifecycleSubsystem {
  private final CANSparkMax motor;
  private RelativeEncoder encoder;
  private SparkMaxPIDController pid;
  private Rotation2d goalAngle = new Rotation2d();

  public WristSubsystem(CANSparkMax motor) {
    super(SubsystemPriority.WRIST);

    this.motor = motor;
    encoder = motor.getEncoder();
    pid = motor.getPIDController();
    motor.setSmartCurrentLimit(35);
    pid.setP(5);
    pid.setI(0);
    pid.setD(0);

    encoder.setPosition(0);
    encoder.setPositionConversionFactor(50);
  }

  @Override
  public void enabledPeriodic() {
    pid.setReference(goalAngle.getRotations(), ControlType.kSmartMotion);
  }

  @Override
  public void robotPeriodic() {
    Logger.getInstance().recordOutput("Wrist/Velocity", encoder.getVelocity());
    Logger.getInstance().recordOutput("Wrist/Angle", getWristAngle().getDegrees());
    Logger.getInstance().recordOutput("Wrist/GoalAngle", goalAngle.getDegrees());
    Logger.getInstance().recordOutput("Wrist/DutyCycleOutput", motor.getAppliedOutput());
    Logger.getInstance().recordOutput("Wrist/StatorCurrent", motor.getOutputCurrent());
  }

  public void set(Rotation2d angle) {
    goalAngle = angle;
  }

  public Command setPositionCommand(Rotation2d angle) {
    return run(() -> {
          set(angle);
        })
        .until(() -> atAngle(angle));
  }

  private boolean atAngle(Rotation2d angle) {
    double actualAngle = getWristAngle().getDegrees();
    return Math.abs(actualAngle - angle.getDegrees()) < 1;
  }

  private Rotation2d getWristAngle() {
    return Rotation2d.fromRotations(encoder.getPosition());
  }
}