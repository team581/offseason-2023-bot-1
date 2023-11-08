// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.Config;
import frc.robot.fms.FmsSubsystem;
import frc.robot.intake.HeldGamePiece;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.managers.autobalance.Autobalance;
import frc.robot.managers.superstructure.NodeHeight;
import frc.robot.managers.superstructure.States;
import frc.robot.managers.superstructure.SuperstructureManager;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.wrist.WristSubsystem;
import java.lang.ref.WeakReference;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Autos {
  private static Command wrapAutoEvent(String commandName, Command command) {
    return Commands.sequence(
            Commands.print("[COMMANDS] Starting auto event " + commandName),
            command.deadlineWith(
                Commands.waitSeconds(5)
                    .andThen(
                        Commands.print(
                            "[COMMANDS] Auto event "
                                + commandName
                                + " has been running for 5+ seconds!"))),
            Commands.print("[COMMANDS] Finished auto event " + commandName))
        .handleInterrupt(() -> System.out.println("[COMMANDS] Cancelled auto event " + commandName))
        .withName(commandName);
  }

  private static Map<String, Command> wrapAutoEventMap(Map<String, Command> eventMap) {
    Map<String, Command> wrappedMap = new HashMap<>();
    for (Map.Entry<String, Command> entry : eventMap.entrySet()) {
      wrappedMap.put(
          entry.getKey(), wrapAutoEvent("AutoEvent_" + entry.getKey(), entry.getValue()));
    }
    return wrappedMap;
  }

  private final LoggedDashboardChooser<AutoKindWithoutTeam> autoChooser =
      new LoggedDashboardChooser<>("Auto Choices");
  private final Map<AutoKind, WeakReference<Command>> autosCache = new EnumMap<>(AutoKind.class);
  private final SwerveAutoBuilder autoBuilder;
  private final LocalizationSubsystem localization;
  private final SwerveSubsystem swerve;
  private final IntakeSubsystem intake;
  private final WristSubsystem wrist;
  private final Autobalance autobalance;
  private final SuperstructureManager superstructure;

  public Autos(
      LocalizationSubsystem localization,
      SwerveSubsystem swerve,
      IntakeSubsystem intake,
      WristSubsystem wrist,
      Autobalance autobalance,
      SuperstructureManager superstructure) {

    this.localization = localization;
    this.swerve = swerve;
    this.intake = intake;
    this.wrist = wrist;
    this.autobalance = autobalance;
    this.superstructure = superstructure;

    Map<String, Command> eventMap =
        Map.ofEntries(
            Map.entry(
                "preloadCube",
                Commands.runOnce(
                    () -> {
                      superstructure.setMode(HeldGamePiece.CUBE);
                      intake.setGamePiece(HeldGamePiece.CUBE);
                    })),
            Map.entry(
                "preloadCone",
                Commands.runOnce(
                    () -> {
                      superstructure.setMode(HeldGamePiece.CONE);
                      intake.setGamePiece(HeldGamePiece.CONE);
                    })),
            Map.entry(
                "scoreLow",
                superstructure
                    .getScoreFinishCommand(() -> NodeHeight.LOW)
                    .withTimeout(1)
                    .andThen(Commands.runOnce(() -> intake.setGamePiece(HeldGamePiece.NOTHING)))),
            Map.entry(
                "scoreMid",
                superstructure
                    .getScoreFinishCommand(() -> NodeHeight.MID)
                    .withTimeout(3)
                    .andThen(Commands.runOnce(() -> intake.setGamePiece(HeldGamePiece.NOTHING)))),
            Map.entry(
                "scoreHigh",
                superstructure
                    .getScoreFinishCommand(() -> NodeHeight.HIGH)
                    .withTimeout(3)
                    .andThen(Commands.runOnce(() -> intake.setGamePiece(HeldGamePiece.NOTHING)))),
            Map.entry(
                "superstructureLow", superstructure.getScoreAlignCommand(() -> NodeHeight.LOW)),
            Map.entry(
                "superstructureMid", superstructure.getScoreAlignCommand(() -> NodeHeight.MID)),
            Map.entry(
                "superstructureHigh", superstructure.getScoreAlignCommand(() -> NodeHeight.HIGH)),
            Map.entry(
                "intakeCone",
                Commands.runOnce(() -> superstructure.setMode(HeldGamePiece.CONE))
                    .andThen(superstructure.getIntakeFloorCommand())),
            Map.entry(
                "intakeCube",
                Commands.runOnce(() -> superstructure.setMode(HeldGamePiece.CUBE))
                    .andThen(superstructure.getIntakeFloorCommand())),
            Map.entry("stow", superstructure.setStateCommand(States.STOWED)),
            Map.entry("stowFast", superstructure.stowFast()));

    eventMap = wrapAutoEventMap(eventMap);

    autoBuilder =
        new SwerveAutoBuilder(
            localization::getPose,
            localization::resetPose,
            SwerveSubsystem.KINEMATICS,
            Config.SWERVE_TRANSLATION_PID,
            Config.SWERVE_ROTATION_PID,
            (states) -> swerve.setModuleStates(states, false, false),
            eventMap,
            false,
            swerve);

    CommandScheduler.getInstance()
        .onCommandInitialize(
            command -> System.out.println("[COMMANDS] Starting command " + command.getName()));
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            command -> System.out.println("[COMMANDS] Cancelled command " + command.getName()));
    CommandScheduler.getInstance()
        .onCommandFinish(
            command -> System.out.println("[COMMANDS] Finished command " + command.getName()));

    for (AutoKindWithoutTeam autoKind : EnumSet.allOf(AutoKindWithoutTeam.class)) {
      autoChooser.addOption(autoKind.toString(), autoKind);
    }

    if (Config.IS_DEVELOPMENT) {
      PathPlannerServer.startServer(5811);
    }

    Logger.getInstance().recordOutput("Autos/CurrentTrajectory", new Trajectory());
    Logger.getInstance().recordOutput("Autos/TargetPose", new Pose2d());
    Logger.getInstance().recordOutput("Autos/SetpointSpeeds/X", 0);
    Logger.getInstance().recordOutput("Autos/SetpointSpeeds/Y", 0);
    Logger.getInstance().recordOutput("Autos/SetpointSpeeds/Omega", 0);
    Logger.getInstance().recordOutput("Autos/TranslationError", new Pose2d());
    Logger.getInstance().recordOutput("Autos/RotationError", 0);

    PPSwerveControllerCommand.setLoggingCallbacks(
        (PathPlannerTrajectory activeTrajectory) -> {
          Logger.getInstance().recordOutput("Autos/CurrentTrajectory", activeTrajectory);
        },
        (Pose2d targetPose) -> {
          Logger.getInstance().recordOutput("Autos/TargetPose", targetPose);
        },
        (ChassisSpeeds setpointSpeeds) -> {
          Logger.getInstance()
              .recordOutput("Autos/SetpointSpeeds/X", setpointSpeeds.vxMetersPerSecond);
          Logger.getInstance()
              .recordOutput("Autos/SetpointSpeeds/Y", setpointSpeeds.vyMetersPerSecond);
          Logger.getInstance()
              .recordOutput("Autos/SetpointSpeeds/Omega", setpointSpeeds.omegaRadiansPerSecond);
        },
        (Translation2d translationError, Rotation2d rotationError) -> {
          Logger.getInstance()
              .recordOutput(
                  "Autos/TranslationError", new Pose2d(translationError, new Rotation2d()));
          Logger.getInstance().recordOutput("Autos/RotationError", rotationError.getDegrees());
        });
  }

  private Command buildAutoCommand(AutoKind auto) {
    WeakReference<Command> ref = autosCache.get(auto);
    if (ref != null && ref.get() != null) {
      Command autoCommand = ref.get();

      if (autoCommand != null) {
        return autoCommand;
      }
    }

    String autoName = "Auto" + auto.toString();
    Command autoCommand = Commands.runOnce(() -> swerve.driveTeleop(0, 0, 0, true, true), swerve);

    if (auto == AutoKind.DO_NOTHING) {
      return autoCommand.andThen(localization.getZeroAwayCommand()).withName(autoName);
    }

    List<PathPlannerTrajectory> pathGroup = Paths.getInstance().getPath(auto);

    autoCommand = autoCommand.andThen(autoBuilder.fullAuto(pathGroup));

    if (auto.autoBalance) {
      autoCommand = autoCommand.andThen(this.autobalance.getCommand());
    }

    autoCommand = autoCommand.withName(autoName);

    autosCache.put(auto, new WeakReference<>(autoCommand));

    return autoCommand;
  }

  public Command getAutoCommand() {
    AutoKindWithoutTeam rawAuto = autoChooser.get();

    if (rawAuto == null) {
      rawAuto = AutoKindWithoutTeam.DO_NOTHING;
    }

    AutoKind auto = FmsSubsystem.isRedAlliance() ? rawAuto.redVersion : rawAuto.blueVersion;

    if (auto == AutoKind.DO_NOTHING) {
      return Commands.none();
    }

    return buildAutoCommand(auto);
  }

  public void clearCache() {
    autosCache.clear();
    Paths.getInstance().clearCache();
  }
}
