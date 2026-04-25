package org.team4639.frc2026.subsystems.fullIntake.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import org.team4639.frc2026.constants.ports.Netherite;
import org.team4639.frc2026.util.PhoenixUtil;
import org.team4639.lib.util.Phoenix6Factory;

public class RollersIOTalonFX implements RollersIO {
  private TalonFX leftLeader;
  private TalonFX rightFollower;

  private TalonFXConfiguration config = new TalonFXConfiguration();

  private VelocityVoltage request = new VelocityVoltage(0);
  private VoltageOut voltageRequest = new VoltageOut(0);

  public RollersIOTalonFX() {
    leftLeader = Phoenix6Factory.createDefaultTalon(Netherite.portConfiguration.intakeLeft);
    rightFollower = Phoenix6Factory.createDefaultTalon(Netherite.portConfiguration.intakeRight);

    config = new TalonFXConfiguration();

    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 20;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Slot0.kV = 0;
    config.Slot0.kS = 0;
    config.Slot0.kA = 0;
    config.Slot0.kP = 0;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;

    PhoenixUtil.tryUntilOk(5, () -> leftLeader.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(5, () -> rightFollower.getConfigurator().apply(config));
    rightFollower.setControl(
        new Follower(
            Netherite.portConfiguration.intakeLeft.getDeviceNumber(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(RollersIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                leftLeader.getStatorCurrent(),
                leftLeader.getMotorVoltage(),
                leftLeader.getDeviceTemp(),
                leftLeader.getVelocity(),
                leftLeader.getPosition())
            .isOK();

    inputs.amps = leftLeader.getStatorCurrent().getValueAsDouble();
    inputs.volts = leftLeader.getMotorVoltage().getValueAsDouble();
    inputs.rotations = leftLeader.getPosition().getValueAsDouble();
    inputs.rotationsPerSeconds = leftLeader.getVelocity().getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    leftLeader.setControl(voltageRequest.withOutput(volts));
  }

  public void setVelocity(double velocity) {
    leftLeader.setControl(request.withVelocity(velocity));
  }

  @Override
  public void stop() {
    leftLeader.stopMotor();
  }
}
