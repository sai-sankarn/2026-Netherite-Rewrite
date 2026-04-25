package org.team4639.frc2026.subsystems.fullIntake.extension;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import org.team4639.frc2026.constants.ports.Netherite;
import org.team4639.frc2026.util.PhoenixUtil;
import org.team4639.lib.util.Phoenix6Factory;

public class ExtensionIOTalonFX implements ExtensionIO {
  private final TalonFX motor;

  private TalonFXConfiguration config = new TalonFXConfiguration();

  private VoltageOut voltageRequest = new VoltageOut(0);

  public ExtensionIOTalonFX() {
    motor = Phoenix6Factory.createDefaultTalon(Netherite.portConfiguration.IntakeExtensionMotorID);

    config.CurrentLimits.StatorCurrentLimit = 15;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 20;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Slot0.kP = 0;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;
    config.Slot0.kS = 0;
    config.Slot0.kV = 0;
    config.Slot0.kA = 0;

    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config));
  }

  @Override
  public void updateInputs(ExtensionIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                motor.getDeviceTemp(),
                motor.getStatorCurrent(),
                motor.getPosition(),
                motor.getVelocity(),
                motor.getMotorVoltage())
            .isOK();

    inputs.volts = motor.getMotorVoltage().getValueAsDouble();
    inputs.amps = motor.getStatorCurrent().getValueAsDouble();
    inputs.temp = motor.getDeviceTemp().getValueAsDouble();
    inputs.position = motor.getPosition().getValueAsDouble();
    inputs.velocity = motor.getVelocity().getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
