// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.Autos;
import frc.robot.config.Config;
import frc.robot.controller.DriveController;
import frc.robot.controller.RumbleControllerSubsystem;
import frc.robot.fms.FmsSubsystem;
import frc.robot.generated.BuildConstants;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.managers.superstructure.SuperstructureManager;
import frc.robot.managers.superstructure.SuperstructureMotionManager;
import frc.robot.shoulder.ShoulderSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystemManager;
import frc.robot.wrist.WristSubsystem;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  // Enables power distribution logging
  private final PowerDistribution pdpLogging = new PowerDistribution(1, ModuleType.kRev);

  private final DriveController driveController = new DriveController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final RumbleControllerSubsystem rumbleController =
      new RumbleControllerSubsystem(new XboxController(1));

  private final FmsSubsystem fmsSubsystem = new FmsSubsystem();
  // TODO: Add all the subsystems & managers here

  private final Autos autos = new Autos();

  private final ShoulderSubsystem shoulder = new ShoulderSubsystem(new CANSparkMax(-1, MotorType.kBrushless), new CANSparkMax(-1, MotorType.kBrushless));
  private final WristSubsystem wrist = new WristSubsystem(new CANSparkMax(-1, MotorType.kBrushless));
  private final IntakeSubsystem intake = new IntakeSubsystem(new CANSparkMax(-1, MotorType.kBrushless));
  private final SuperstructureMotionManager motionManager = new SuperstructureMotionManager(shoulder, wrist);
  private final SuperstructureManager superstructureManager = new SuperstructureManager(motionManager, intake);

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
    // TODO: Start adding button bindings

    //Driver controls

    //Floor intake
    //driveController.leftTrigger(0.3).onTrue();

    //Set mode to cubes


    //Operator controls
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
