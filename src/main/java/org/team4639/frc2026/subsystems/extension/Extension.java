/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.extension;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.util.Phoenix6Factory;
import org.team4639.lib.util.PhoenixUtil;

public class Extension extends SubsystemBase {

  private final TalonFX extensionMotor;

  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  // Change these to whatever works for your mechanism.
  private static final double MAX_VELOCITY = 20.0; // rotations/sec
  private static final double MAX_ACCELERATION = 40.0; // rotations/sec^2

  // Position tolerance in motor rotations.
  private static final double POSITION_TOLERANCE = 0.05;

  private static final double RETRACTED_POSITION = -6.997559;
  private static final double EXTENDED_POSITION = 13.665527;

  public Extension(PortConfiguration ports) {
    extensionMotor = Phoenix6Factory.createDefaultTalon(ports.IntakeExtensionMotorID, false);

    // Current limits
    config.CurrentLimits.SupplyCurrentLimit = 20;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.CurrentLimits.StatorCurrentLimit = 15;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // Motion Magic settings
    config.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = MAX_ACCELERATION;

    PhoenixUtil.tryUntilOk(5, () -> extensionMotor.getConfigurator().apply(config));

    extensionMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  /** Extend using open-loop voltage. */
  public void extend() {
    extensionMotor.setControl(voltageRequest.withOutput(4.0));
  }

  /** Retract using open-loop voltage. */
  public void retract() {
    extensionMotor.setControl(voltageRequest.withOutput(-3.0));
  }

  /** Move to a specific motor rotation position using Motion Magic. */
  public void setPosition(double position) {
    extensionMotor.setControl(motionMagicRequest.withPosition(position));
  }

  /** Command that moves the extension to a position and finishes when it reaches that position. */
  public Command moveToPosition(double position) {
    return Commands.run(() -> setPosition(position), this)
        .until(() -> Math.abs(getPosition() - position) < POSITION_TOLERANCE)
        .andThen(this::stop);
  }

  /**
   * Command to fully extend.
   *
   * <p>Replace this with your actual maximum safe motor position.
   */
  public Command extendToEnd() {
    return moveToPosition(EXTENDED_POSITION);
  }

  /**
   * Command to fully retract.
   *
   * <p>Replace this with your actual minimum safe motor position.
   */
  public Command retractToEnd() {
    return moveToPosition(RETRACTED_POSITION);
  }

  /** Run the motor at a specific voltage. */
  public void setVoltage(double voltage) {
    extensionMotor.setControl(voltageRequest.withOutput(voltage));
  }

  /** Stop the extension motor. */
  public void stop() {
    extensionMotor.stopMotor();
  }

  /** Set the motor to brake or coast. */
  public void setBrakeMode(boolean brake) {
    extensionMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  /** Get the current motor position in rotations. */
  public double getPosition() {
    return extensionMotor.getPosition().getValueAsDouble();
  }

  /** Get motor velocity in rotations per second. */
  public double getVelocity() {
    return extensionMotor.getVelocity().getValueAsDouble();
  }

  /** Get motor current in amps. */
  public double getCurrent() {
    return extensionMotor.getStatorCurrent().getValueAsDouble();
  }

  /** Get motor voltage. */
  public double getVoltage() {
    return extensionMotor.getMotorVoltage().getValueAsDouble();
  }
}
