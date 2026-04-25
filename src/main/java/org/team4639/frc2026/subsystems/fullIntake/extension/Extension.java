package org.team4639.frc2026.subsystems.fullIntake.extension;

import org.littletonrobotics.junction.Logger;
import org.team4639.lib.util.FullSubsystem;

import lombok.Setter;

public class Extension extends FullSubsystem {
  private final ExtensionIO extensionIO;
  private ExtensionIOInputsAutoLogged extensioIOInputsAutoLogged;

  public enum WantedState{
    EXTENDED,
    RETRACTED,
    IDLE
  }

  public enum SystemState{
    EXTENDED,
    RETRACTED,
    MOVING_IN,
    MOVING_OUT,
    IDLE
  }

  private SystemState systemState = SystemState.IDLE;
  @Setter private WantedState wantedState = WantedState.IDLE;

  public Extension(ExtensionIO extensionIO) {
    this.extensionIO = extensionIO;
  }

  public void setVoltage(double volts){
    extensionIO.setVoltage(volts);
  }

  @Override
  public void periodicBeforeScheduler(){
    extensionIO.updateInputs(extensioIOInputsAutoLogged);
    Logger.processInputs("Extension Inputs", extensioIOInputsAutoLogged);
  }

  public void runStateMachine(){

  }

  public void handleStateTransition(){
    
  }

}
















