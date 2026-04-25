package org.team4639.frc2026.subsystems.fullIntake.rollers;

import org.littletonrobotics.junction.Logger;
import org.team4639.lib.util.FullSubsystem;

import lombok.Setter;

public class Rollers extends FullSubsystem {
  private final RollersIO rollersIO;
  private final RollersIOInputsAutoLogged rollerInputs = new RollersIOInputsAutoLogged();

  private final double ROLLER_VOLTAGE = 0;

  public enum SystemState{
    INTAKE,
    OUTTAKE,
    IDLE
  }

  public enum WantedState{
    INTAKE,
    OUTTAKE,
    IDLE
  }

  private SystemState systemState = SystemState.IDLE;
  @Setter private WantedState wantedState = WantedState.IDLE;

  public Rollers(RollersIO rollers) {
    this.rollersIO = rollers;
    rollersIO.updateInputs(rollerInputs);
    setDefaultCommand(run(this::runStateMachine));
  }

  public void setVoltage(double volts) {
    rollersIO.setVoltage(volts);
  }

  @Override
  public void periodicBeforeScheduler() {
    rollersIO.updateInputs(rollerInputs);
    Logger.processInputs("Rollers Inputs", rollerInputs);
  }

  public void runStateMachine(){
    SystemState newState = handleStateTransition();
    if (newState != systemState){
      systemState = newState;
    }
    switch (systemState){
      case INTAKE -> handleIntake();
      case OUTTAKE -> handleOuttake();
      case IDLE -> handleIdle();
    }
  }

  public void handleIntake(){
    rollersIO.setVoltage(ROLLER_VOLTAGE);
  }

  public void handleOuttake(){
    rollersIO.setVoltage(-ROLLER_VOLTAGE);
  }

  public void handleIdle(){
    rollersIO.stop();
  }

  public SystemState handleStateTransition(){
    return switch (wantedState){
      case INTAKE -> SystemState.INTAKE;
      case OUTTAKE -> SystemState.OUTTAKE;
      case IDLE -> SystemState.OUTTAKE;
    };
  }
}
