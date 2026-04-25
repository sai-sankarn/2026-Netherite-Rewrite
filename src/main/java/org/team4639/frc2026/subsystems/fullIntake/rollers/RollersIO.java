package org.team4639.frc2026.subsystems.fullIntake.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {

  default void setVoltage(double volts) {}

  default void setVelocity(double velocity) {}

  default void stop() {}

  default void updateInputs(RollersIOInputs inputs) {}

  @AutoLog
  class RollersIOInputs {
    public boolean connected = true;
    public double volts;
    public double amps;
    public double rotationsPerSeconds;
    public double rotations;
    public double temperature;
  }
}
