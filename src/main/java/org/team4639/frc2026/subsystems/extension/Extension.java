/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.extension;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team4639.frc2026.util.PortConfiguration;
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
  private static final double POSITION_TOLERANCE = 1;

  private static final double RETRACTED_POSITION = 0;
  private static final double EXTENDED_POSITION = 18.9287109375;

  public Extension(PortConfiguration ports) {
    extensionMotor = new TalonFX(ports.IntakeExtensionMotorID.getDeviceNumber());

    // Current limits
    config.CurrentLimits.SupplyCurrentLimit = 20;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.CurrentLimits.StatorCurrentLimit = 15;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // Motion Magic settings
    config.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = MAX_ACCELERATION;

    PhoenixUtil.tryUntilOk(5, () -> extensionMotor.getConfigurator().apply(config));

    extensionMotor.setNeutralMode(NeutralModeValue.Coast);
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
  public Command moveToPosition(double position, boolean extending) {
    if (extending) {
      return Commands.run(this::extend, this)
          .until(() -> Math.abs(getPosition() - position) < POSITION_TOLERANCE)
          .andThen(this::stop);
    } else {
      return Commands.run(this::retract, this)
          .until(() -> Math.abs(getPosition() - position) < POSITION_TOLERANCE)
          .andThen(this::stop);
    }
  }

  public Command home() {
    return Commands.run(this::retract, this)
        .until(() -> getCurrent() >= 10)
        .withTimeout(3.0)
        .andThen(
            Commands.runOnce(
                () -> {
                  stop();
                  extensionMotor.setPosition(0.0);
                },
                this));
  }

  /**
   * Command to fully extend.
   *
   * <p>Replace this with your actual maximum safe motor position.
   */
  public Command extendToEnd() {
    return moveToPosition(EXTENDED_POSITION, true);
  }

  /**
   * Command to fully retract.
   *
   * <p>Replace this with your actual minimum safe motor position.
   */
  public Command retractToEnd() {
    return moveToPosition(RETRACTED_POSITION, false);
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

  public void periodic() {
    SmartDashboard.putNumber("Extension Position", this.getPosition());
    SmartDashboard.putNumber("Extension Goal Difference", getPosition() - EXTENDED_POSITION);
    SmartDashboard.putBoolean(
        "Extended", Math.abs(getPosition() - EXTENDED_POSITION) < POSITION_TOLERANCE);
  }
}
