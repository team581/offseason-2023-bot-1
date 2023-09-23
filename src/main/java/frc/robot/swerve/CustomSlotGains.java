package frc.robot.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;

class CustomSlotGains extends Slot0Configs {
  public CustomSlotGains(double kP, double kI, double kD, double kV, double kS) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kV = kV;
        this.kS = kS;
  }
}
