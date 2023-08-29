package frc.robot.lights;


import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.fms.FmsSubsystem;
import frc.robot.intake.HeldGamePiece;
import frc.robot.intake.IntakeState;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.managers.superstructure.SuperstructureManager;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class LightsSubsystem extends LifecycleSubsystem{

  private static final double FAST_BLINK_DURATION = 0.08;
  private static final double SLOW_BLINK_DURATION = 0.25;


  private final CANdle candle;
  private final IntakeSubsystem intake;
  private final SuperstructureManager superstructure;

  private final Timer blinkTimer = new Timer();
  private Color color = Color.kWhite;
  // TODO: Copy paste blink pattern enum from other repo
  private BlinkPattern blinkPattern = BlinkPattern.SOLID;

  public LightsSubsystem(
    CANdle candle,
    IntakeSubsystem intake,
    SuperstructureManager superstructure
  ) {
    super(SubsystemPriority.LIGHTS);
    this.candle = candle;
    this.intake = intake;
    this.superstructure = superstructure;
  }
  public void enabledPeriodic(){
    HeldGamePiece gamePiece = intake.getGamePiece();
    IntakeState intakeMode = intake.getIntakeState();
    HeldGamePiece superstructureMode = superstructure.getMode();

    if (DriverStation.isDisabled()) {
      if (FmsSubsystem.isRedAlliance()) {
        color = Color.kRed;
        blinkPattern = BlinkPattern.SOLID;
      } else {
        color = Color.kBlue;
        blinkPattern = BlinkPattern.SOLID;
      }
    }else if (gamePiece == HeldGamePiece.CUBE) {
      if (intakeMode == IntakeState.INTAKE_CUBE) {
        color = Color.kPurple;
        blinkPattern = BlinkPattern.BLINK_FAST;
      } else {
        color = Color.kPurple;
        blinkPattern = BlinkPattern.SOLID;
      }
    } else if (gamePiece == HeldGamePiece.CONE) {
      if (intakeMode == IntakeState.INTAKE_CONE) {
        color = Color.kYellow;
        blinkPattern = BlinkPattern.BLINK_FAST;
      } else {
        color = Color.kYellow;
        blinkPattern = BlinkPattern.SOLID;
      }
    } else if (superstructureMode == HeldGamePiece.CUBE) {
      color = Color.kPurple;
      blinkPattern = BlinkPattern.BLINK_SLOW;
    } else if (superstructureMode == HeldGamePiece.CONE) {
      color = Color.kYellow;
      blinkPattern = BlinkPattern.BLINK_SLOW;
    } else {
      color = Color.kWhite;
      blinkPattern = BlinkPattern.SOLID;
    }

    Color8Bit color8Bit = new Color8Bit(color);
    if (blinkPattern == BlinkPattern.SOLID) {
      candle.setLEDs(color8Bit.red, color8Bit.green, color8Bit.blue);
    } else {
      double time = blinkTimer.get();
      double onDuration = 0;
      double offDuration = 0;

      if (blinkPattern == BlinkPattern.BLINK_FAST) {
        onDuration = FAST_BLINK_DURATION;
        offDuration = FAST_BLINK_DURATION * 2;
      } else if (blinkPattern == BlinkPattern.BLINK_SLOW) {
        onDuration = SLOW_BLINK_DURATION;
        offDuration = SLOW_BLINK_DURATION * 2;
      }

      if (time >= offDuration) {
        blinkTimer.reset();
        candle.setLEDs(0, 0, 0);
      } else if (time >= onDuration) {
        candle.setLEDs(color8Bit.red, color8Bit.green, color8Bit.blue);
      }
    }


  }



}
