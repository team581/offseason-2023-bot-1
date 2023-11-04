// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.imu.ImuSubsystem;
import frc.robot.intake.HeldGamePiece;
import frc.robot.intake.IntakeState;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.shoulder.ShoulderSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.wrist.WristSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SuperstructureManager extends LifecycleSubsystem {
  private final ImuSubsystem imu;
  private final SuperstructureMotionManager motionManager;
  private final ShoulderSubsystem shoulder;
  private final WristSubsystem wrist;
  private final IntakeSubsystem intake;
  private SuperstructureState goalState = States.STOWED;
  private HeldGamePiece mode = HeldGamePiece.CUBE;
  private NodeHeight scoringHeight = null;
  private IntakeState manualIntakeState = null;
  private ScoringProgress scoringProgress = ScoringProgress.NOT_SCORING;

  public SuperstructureManager(
      SuperstructureMotionManager motionManager, IntakeSubsystem intake, ImuSubsystem imu) {
    super(SubsystemPriority.SUPERSTRUCTURE_MANAGER);

    this.motionManager = motionManager;
    this.shoulder = motionManager.shoulder;
    this.wrist = motionManager.wrist;
    this.intake = intake;
    this.imu = imu;
  }

  private void setGoal(SuperstructureState goalState) {
    this.goalState = goalState;
    if (goalState.equals(States.STOWED)) {
      scoringProgress = ScoringProgress.NOT_SCORING;
    }
  }

  public boolean atGoal(SuperstructureState state) {
    return motionManager.atPosition(state.position) && intake.atGoal(state.intakeState);
  }

  @Override
  public void enabledPeriodic() {
    motionManager.set(goalState.position);

    if (manualIntakeState != null) {
      intake.setGoalState(manualIntakeState);
    } else if (motionManager.atPosition(goalState.position) || goalState.intakeNow) {
      intake.setGoalState(goalState.intakeState);
    }
  }

  @Override
  public void robotPeriodic() {
    Logger.getInstance()
        .recordOutput(
            "Superstructure/GoalState/GoalWristAngle", goalState.position.wristAngle.getDegrees());
    Logger.getInstance()
        .recordOutput(
            "Superstructure/GoalState/GoalShoulderAngle",
            goalState.position.shoulderAngle.getDegrees());
    Logger.getInstance()
        .recordOutput("Superstructure/GoalState/GoalIntakeState", goalState.intakeState.toString());
    Logger.getInstance()
        .recordOutput("Superstructure/GoalState/GoalIntakeNow", goalState.intakeNow);
    Logger.getInstance().recordOutput("Superstructure/Mode", mode.toString());
  }

  public void setIntakeOverride(IntakeState intakeState) {
    manualIntakeState = intakeState;
  }

  public Command setIntakeOverrideCommand(IntakeState intakeState) {
    return runOnce(
            () -> {
              setIntakeOverride(intakeState);
            })
        .withName("IntakeOverrideCommand");
  }

  public Command setStateCommand(SuperstructureState newGoalState) {
    return setStateCommand(() -> newGoalState).withName("SetStateCommand");
  }

  public Command setStateCommand(Supplier<SuperstructureState> newGoalState) {
    return Commands.runOnce(() -> setGoal(newGoalState.get()), wrist, shoulder)
        .andThen(Commands.waitUntil(() -> atGoal(newGoalState.get())))
        .withName("SetStateCommand");
  }

  public void setMode(HeldGamePiece mode) {
    this.mode = mode;
  }

  public Command setModeCommand(HeldGamePiece mode) {
    return runOnce(
            () -> {
              setMode(mode);
            })
        .withName("SetModeCommand");
  }

  public HeldGamePiece getMode() {
    return mode;
  }

  public ScoringProgress getScoringProgress() {
    return scoringProgress;
  }

  public Command getIntakeFloorCommand() {
    return setStateCommand(
            () -> {
              if (mode == HeldGamePiece.CONE) {
                return States.INTAKING_CONE_FLOOR;
              } else {
                return States.INTAKING_CUBE_FLOOR;
              }
            })
        .andThen(stowFast())
        .withName("IntakeFloorCommand");
  }

  public Command getIntakeShelfCommand() {
    return setStateCommand(
            () -> {
              if (mode == HeldGamePiece.CONE) {
                return States.INTAKING_CONE_SHELF;
              } else {
                return States.INTAKING_CUBE_SHELF;
              }
            })
        .andThen(stowFast())
        .withName("IntakeShelfCommand");
  }

  private Command stowFast() {
    return Commands.runOnce(
        () -> {
          setGoal(States.STOWED);
        },
        wrist,
        shoulder);
  }

  public Command getIntakeSingleSubstationCommand() {
    return setStateCommand(States.INTAKING_CONE_SINGLE_SUBSTATION)
        .andThen(stowFast())
        .withName("IntakeSingleSubstationCommand");
  }

  private SuperstructureScoringState getScoringState(NodeHeight height) {
    SuperstructureScoringState cubeState;
    SuperstructureScoringState coneState;

    if (height == NodeHeight.LOW) {
      // double heading = imu.getRobotHeading().getDegrees();
      // double leftAngle = AutoRotate.getLeftAngle().getDegrees();
      // double rightAngle = AutoRotate.getRightAngle().getDegrees();
      // if (heading < leftAngle && heading > rightAngle) {
      //   cubeState = States.CUBE_NODE_LOW_FRONT;
      //   coneState = States.CONE_NODE_LOW_FRONT;
      // } else {
      //   cubeState = States.CUBE_NODE_LOW_BACK;
      //   coneState = States.CONE_NODE_LOW_BACK;
      // }
      cubeState = States.CUBE_NODE_LOW_BACK;
      coneState = States.CONE_NODE_LOW_BACK;
    } else if (height == NodeHeight.MID) {
      cubeState = States.CUBE_NODE_MID;
      coneState = States.CONE_NODE_MID;
    } else {
      cubeState = States.CUBE_NODE_HIGH;
      coneState = States.CONE_NODE_HIGH;
    }

    if (mode == HeldGamePiece.CONE) {
      return coneState;
    } else {
      return cubeState;
    }
  }

  public Command getScoreAlignCommand(NodeHeight height) {
    return Commands.runOnce(
            () -> {
              scoringHeight = height;
              scoringProgress = ScoringProgress.ALIGNING;
            })
        .andThen(setStateCommand(getScoringState(height).aligning))
        .withName("ScoreAlignCommand");
  }

  public Command getScoreFinishCommand() {
    return getScoreFinishCommand(scoringHeight == null ? NodeHeight.LOW : scoringHeight)
        .withName("ScoreFinishCommand");
  }

  public Command getScoreFinishCommand(NodeHeight height) {
    return Commands.runOnce(
            () -> {
              scoringHeight = height;
              scoringProgress = ScoringProgress.PLACING;
            })
        .andThen(setStateCommand(getScoringState(height).aligning))
        .andThen(setStateCommand(getScoringState(height).scoring))
        .andThen(
            Commands.runOnce(
                () -> {
                  scoringHeight = null;
                  scoringProgress = ScoringProgress.DONE_SCORING;
                }))
        .andThen(stowFast().unless(() -> height != NodeHeight.LOW))
        .withName("ScoreFinishCommand");
  }
}
