package org.team4639.frc2026.subsystems.fullIntake.extension;

import org.littletonrobotics.junction.AutoLog;

public interface ExtensionIO {
  default void setVoltage(double volts) {}

  default void stop() {}

  default void updateInputs(ExtensionIOInputs inputs) {}

  @AutoLog
  class ExtensionIOInputs {
    public boolean connected = true;
    public double temp;
    public double amps;
    public double position;
    public double volts;
    public double velocity;
  }
}
