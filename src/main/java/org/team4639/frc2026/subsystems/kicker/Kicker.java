package org.team4639.frc2026.subsystems.kicker;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team4639.frc2026.util.PortConfiguration;

public class Kicker extends SubsystemBase {
  private final TalonFX motor;

  public Kicker(PortConfiguration ports) {
    motor = new TalonFX(ports.KickerMotorID.getDeviceNumber());
  }

  public Command runKicker() {
    return run(() -> motor.setVoltage(6));
  }

  public Command idle() {
    return run(() -> motor.stopMotor());
  }
}
