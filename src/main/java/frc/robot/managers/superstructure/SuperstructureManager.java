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
import frc.robot.managers.autorotate.AutoRotate;
import frc.robot.shoulder.ShoulderSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.wrist.WristSubsystem;
import java.util.function.Supplier;

public class SuperstructureManager extends LifecycleSubsystem {
  private final ImuSubsystem imu;
  private final SuperstructureMotionManager motionManager;
  private final ShoulderSubsystem shoulder;
  private final WristSubsystem wrist;
  private final IntakeSubsystem intake;
  private SuperstructureState goalState;
  private HeldGamePiece mode = HeldGamePiece.CUBE;
  private NodeHeight scoringHeight = null;
  private IntakeState manualIntakeState = null;

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

  public void setIntakeOverride(IntakeState intakeState) {
    manualIntakeState = intakeState;
  }

  public Command setIntakeOverrideCommand(IntakeState intakeState) {
    return runOnce(
        () -> {
          setIntakeOverride(intakeState);
        });
  }

  public Command setStateCommand(SuperstructureState newGoalState) {
    return setStateCommand(() -> newGoalState);
  }

  public Command setStateCommand(Supplier<SuperstructureState> newGoalState) {
    return Commands.runOnce(() -> setGoal(newGoalState.get()))
        .andThen(Commands.waitUntil(() -> atGoal(newGoalState.get())));
  }

  public void setMode(HeldGamePiece mode) {
    this.mode = mode;
  }

  public Command setModeCommand(HeldGamePiece mode) {
    return runOnce(
        () -> {
          setMode(mode);
        });
  }

  public HeldGamePiece getMode() {
    return mode;
  }

  public Command getIntakeFloorCommand() {
    return setStateCommand(
        () -> {
          if (mode == HeldGamePiece.CONE) {
            return States.INTAKING_CONE_FLOOR;
          } else {
            return States.INTAKING_CUBE_FLOOR;
          }
        });
  }

  public Command getIntakeShelfCommand() {
    return setStateCommand(
        () -> {
          if (mode == HeldGamePiece.CONE) {
            return States.INTAKING_CONE_SHELF;
          } else {
            return States.INTAKING_CUBE_SHELF;
          }
        });
  }

  public Command getIntakeSingleSubstationCommand() {
    return setStateCommand(States.INTAKING_CONE_SINGLE_SUBSTATION);
  }

  private SuperstructureScoringState getScoringState(NodeHeight height) {
    SuperstructureScoringState cubeState;
    SuperstructureScoringState coneState;

    if (height == NodeHeight.LOW) {
      double heading = imu.getRobotHeading().getDegrees();
      double leftAngle = AutoRotate.getLeftAngle().getDegrees();
      double rightAngle = AutoRotate.getRightAngle().getDegrees();
      if (heading < leftAngle && heading > rightAngle) {
        cubeState = States.CUBE_NODE_LOW_FRONT;
        coneState = States.CONE_NODE_LOW_FRONT;
      } else {
        cubeState = States.CUBE_NODE_LOW_BACK;
        coneState = States.CONE_NODE_LOW_BACK;
      }

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
            })
        .andThen(setStateCommand(getScoringState(height).aligning));
  }

  public Command getScoreFinishCommand() {
    return getScoreFinishCommand(scoringHeight == null ? NodeHeight.LOW : scoringHeight);
  }

  public Command getScoreFinishCommand(NodeHeight height) {
    return Commands.runOnce(
            () -> {
              scoringHeight = height;
            })
        .andThen(setStateCommand(getScoringState(height).scoring))
        .andThen(
            Commands.runOnce(
                () -> {
                  scoringHeight = null;
                }));
  }

  public Command yeetConeCommand() {
    return Commands.runOnce(() -> (setStateCommand(States.YEET_CONE))).waitUntil(atGoal(States.YEET_CONE)).andThen(setStateCommand(States.STOWED));
  }
}
