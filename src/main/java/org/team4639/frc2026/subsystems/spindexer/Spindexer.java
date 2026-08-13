/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.spindexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.util.Phoenix6Factory;
import org.team4639.lib.util.PhoenixUtil;

public class Spindexer extends SubsystemBase {

  private final TalonFX spindexerMotor;

  private final VelocityVoltage velocityControl = new VelocityVoltage(0);

  private final VoltageOut voltageControl = new VoltageOut(0);

  // Normal operating speed
  private static final double SPIN_RPM = 1800.0;

  // Feedforward values from the original subsystem
  private static final double KV = 0.087712 * 9.0 / 8.0;
  private static final double KA = 0.23735;

  public Spindexer(PortConfiguration ports) {

    spindexerMotor = Phoenix6Factory.createDefaultTalon(ports.SpindexerMotorID, false);

    TalonFXConfiguration config = new TalonFXConfiguration();

    // Motor behavior
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.MotorOutput.PeakForwardDutyCycle = 1.0;
    config.MotorOutput.PeakReverseDutyCycle = 0.0;

    // Current limits
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 35;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;

    // Velocity feedforward
    config.Slot0.kV = KV;
    config.Slot0.kA = KA;
    config.Slot0.kP = 0.0;

    PhoenixUtil.tryUntilOk(5, () -> spindexerMotor.getConfigurator().apply(config));

    setDefaultCommand(stopCommand());
  }

  /** Spin the spindexer at the normal operating speed. */
  public void spin() {
    setRPM(SPIN_RPM);
  }

  /**
   * Spin the spindexer at a specific RPM.
   *
   * <p>The TalonFX expects rotations per second at the rotor, so RPM is converted using the same
   * calculation as the original subsystem.
   */
  public void setRPM(double rpm) {

    double rotationsPerSecond = rpm * 4.0 / 60.0;

    spindexerMotor.setControl(velocityControl.withVelocity(rotationsPerSecond));
  }

  /** Run the motor at a specific voltage. */
  public void setVoltage(double voltage) {
    spindexerMotor.setControl(voltageControl.withOutput(voltage));
  }

  /** Stop the spindexer. */
  public void stop() {
    spindexerMotor.stopMotor();
  }

  /** Get the motor velocity in RPM. */
  public double getRPM() {

    double rotationsPerSecond = spindexerMotor.getVelocity().getValueAsDouble();

    return rotationsPerSecond * 60.0 / 4.0;
  }

  /** Get the motor current in amps. */
  public double getCurrent() {
    return spindexerMotor.getStatorCurrent().getValueAsDouble();
  }

  /** Get the motor voltage. */
  public double getVoltage() {
    return spindexerMotor.getMotorVoltage().getValueAsDouble();
  }

  /** Get the motor position in rotations. */
  public double getPosition() {
    return spindexerMotor.getPosition().getValueAsDouble();
  }

  public Command spinCommand() {
    return run(this::spin);
  }

  public Command stopCommand() {
    return run(this::stop);
  }
}
