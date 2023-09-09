// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.Autos;
import frc.robot.config.Config;
import frc.robot.controller.DriveController;
import frc.robot.controller.RumbleControllerSubsystem;
import frc.robot.fms.FmsSubsystem;
import frc.robot.generated.BuildConstants;
import frc.robot.imu.ImuSubsystem;
import frc.robot.intake.HeldGamePiece;
import frc.robot.intake.IntakeState;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.managers.autobalance.Autobalance;
import frc.robot.managers.autorotate.AutoRotate;
import frc.robot.managers.superstructure.NodeHeight;
import frc.robot.managers.superstructure.States;
import frc.robot.managers.superstructure.SuperstructureManager;
import frc.robot.managers.superstructure.SuperstructureMotionManager;
import frc.robot.shoulder.ShoulderSubsystem;
import frc.robot.swerve.SwerveModule;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystemManager;
import frc.robot.wrist.WristSubsystem;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  // Enables power distribution logging
  private final PowerDistribution pdpLogging =
      new PowerDistribution(Config.PDP_ID, ModuleType.kRev);
  private final SwerveModule frontLeft =
      new SwerveModule(
          Config.SWERVE_FL_CONSTANTS,
          new TalonFX(Config.SWERVE_FL_DRIVE_MOTOR_ID, Config.CANIVORE_ID),
          new TalonFX(Config.SWERVE_FL_STEER_MOTOR_ID, Config.CANIVORE_ID),
          new CANcoder(Config.SWERVE_FL_CANCODER_ID, Config.CANIVORE_ID));
  private final SwerveModule frontRight =
      new SwerveModule(
          Config.SWERVE_FR_CONSTANTS,
          new TalonFX(Config.SWERVE_FR_DRIVE_MOTOR_ID, Config.CANIVORE_ID),
          new TalonFX(Config.SWERVE_FR_STEER_MOTOR_ID, Config.CANIVORE_ID),
          new CANcoder(Config.SWERVE_FR_CANCODER_ID, Config.CANIVORE_ID));
  private final SwerveModule backLeft =
      new SwerveModule(
          Config.SWERVE_BL_CONSTANTS,
          new TalonFX(Config.SWERVE_BL_DRIVE_MOTOR_ID, Config.CANIVORE_ID),
          new TalonFX(Config.SWERVE_BL_STEER_MOTOR_ID, Config.CANIVORE_ID),
          new CANcoder(Config.SWERVE_BL_CANCODER_ID, Config.CANIVORE_ID));
  private final SwerveModule backRight =
      new SwerveModule(
          Config.SWERVE_BR_CONSTANTS,
          new TalonFX(Config.SWERVE_BR_DRIVE_MOTOR_ID, Config.CANIVORE_ID),
          new TalonFX(Config.SWERVE_BR_STEER_MOTOR_ID, Config.CANIVORE_ID),
          new CANcoder(Config.SWERVE_BR_CANCODER_ID, Config.CANIVORE_ID));

  private final DriveController driveController = new DriveController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final RumbleControllerSubsystem rumbleController =
      new RumbleControllerSubsystem(new XboxController(1));

  private final FmsSubsystem fmsSubsystem = new FmsSubsystem();

  private final Autos autos = new Autos();

  private final ShoulderSubsystem shoulder =
      new ShoulderSubsystem(
          new CANSparkMax(Config.SHOULDER_ID, MotorType.kBrushless),
          new CANSparkMax(Config.SHOULDER2_ID, MotorType.kBrushless));
  private final WristSubsystem wrist =
      new WristSubsystem(new CANSparkMax(Config.WRIST_ID, MotorType.kBrushless));
  private final IntakeSubsystem intake =
      new IntakeSubsystem(new CANSparkMax(Config.INTAKE_ID, MotorType.kBrushless));
  private final SuperstructureMotionManager motionManager =
      new SuperstructureMotionManager(shoulder, wrist);
  private final ImuSubsystem imu =
      new ImuSubsystem(new Pigeon2(Config.PIGEON2_ID, Config.CANIVORE_ID));
  private final SuperstructureManager superstructure =
      new SuperstructureManager(motionManager, intake, imu);
  private final SwerveSubsystem swerve =
      new SwerveSubsystem(imu, frontRight, frontLeft, backRight, backLeft);

  private final Autobalance autobalance = new Autobalance(swerve, imu);
  private final AutoRotate autoRotate = new AutoRotate(swerve);

  private Command autoCommand;

  public Robot() {
    // Log to a USB stick
    Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1/"));
    if (Config.IS_DEVELOPMENT) {
      // Publish data to NetworkTables
      Logger.getInstance().addDataReceiver(new NT4Publisher());
    }

    // Record metadata
    Logger.getInstance().recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.getInstance().recordMetadata("RoborioSerialNumber", Config.SERIAL_NUMBER);
    Logger.getInstance().recordMetadata("RobotConfig", "Offseason bot 1");
    Logger.getInstance().recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.getInstance().recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.getInstance().recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.getInstance().recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.getInstance().recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.getInstance().recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.getInstance().recordMetadata("GitDirty", "Unknown");
        break;
    }

    Logger.getInstance().start();

    // This must be run before any commands are scheduled
    LifecycleSubsystemManager.getInstance().ready();

    configureButtonBindings();

    enableLiveWindowInTest(false);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {}

  private void configureButtonBindings() {
    swerve.setDefaultCommand(swerve.getDriveTeleopCommand(driveController));

    // TODO: Start adding button bindings

    // Driver controls
    // driveController.leftStick();
    // driveController.rightStick();
    driveController.leftTrigger(0.3).onTrue(superstructure.getIntakeFloorCommand());
    driveController.leftBumper().onTrue(superstructure.getIntakeShelfCommand());
    driveController.rightTrigger(0.3).onTrue(superstructure.getScoreFinishCommand());
    driveController.rightBumper().onTrue(superstructure.getIntakeSingleSubstationCommand());
    driveController.back().onTrue(imu.getZeroCommand());

    driveController.povUp().onTrue(superstructure.setModeCommand(HeldGamePiece.CUBE));
    driveController.povDown().onTrue(superstructure.setModeCommand(HeldGamePiece.CONE));
    // Snaps for all cardinal directions
    driveController.x().onTrue(autoRotate.getCommand(() -> AutoRotate.getLeftAngle()));
    driveController.b().onTrue(autoRotate.getCommand(() -> AutoRotate.getRightAngle()));
    driveController.y().onTrue(autoRotate.getCommand(() -> AutoRotate.getForwardAngle()));
    driveController.a().onTrue(autoRotate.getCommand(() -> AutoRotate.getBackwardsAngle()));

    new Trigger(() -> driveController.getThetaPercentage() == 0)
        .onFalse(autoRotate.getDisableCommand());
    // X swerve
    driveController.start().onTrue(swerve.getXSwerveCommand());

    new Trigger(
            () ->
                driveController.getSidewaysPercentage() == 0
                    && driveController.getForwardPercentage() == 0
                    && driveController.getThetaPercentage() == 0)
        .onFalse(swerve.disableXSwerveCommand());

    // Operator controls
    operatorController.y().onTrue(superstructure.getScoreAlignCommand(NodeHeight.HIGH));
    operatorController.b().onTrue(superstructure.getScoreAlignCommand(NodeHeight.MID));
    operatorController.a().onTrue(superstructure.setStateCommand(States.STOWED));

    // Manual intake override
    operatorController
        .leftTrigger(0.3)
        .onTrue(superstructure.setIntakeOverrideCommand(IntakeState.MANUAL_INTAKE))
        .onFalse(superstructure.setIntakeOverrideCommand(null));
    operatorController
        .rightTrigger(0.3)
        .onTrue(superstructure.setIntakeOverrideCommand(IntakeState.MANUAL_OUTTAKE))
        .onFalse(superstructure.setIntakeOverrideCommand(null));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    LifecycleSubsystemManager.getInstance().log();
  }

  @Override
  public void autonomousInit() {
    autoCommand = autos.getAutoCommand();
    CommandScheduler.getInstance().schedule(autoCommand);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (autoCommand != null) {
      autoCommand.cancel();
      autoCommand = null;
    }

    autos.clearCache();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    // Keep selected auto in cache to avoid loading it when auto starts
    autos.getAutoCommand();
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
