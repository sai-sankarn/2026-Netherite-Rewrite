package org.team4639.frc2026.subsystems.rollers;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team4639.frc2026.util.PortConfiguration;

public class Rollers extends SubsystemBase {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  public Rollers(PortConfiguration ports) {
    leftMotor = new TalonFX(ports.intakeLeft.getDeviceNumber());
    rightMotor = new TalonFX(ports.intakeRight.getDeviceNumber());

    rightMotor.setControl(
        new Follower(ports.intakeLeft.getDeviceNumber(), MotorAlignmentValue.Opposed));
  }

  public void handleIntaking() {
    leftMotor.setVoltage(6);
  }

  public void handleOuttaking() {
    leftMotor.setVoltage(-6);
  }

  public void stop() {
    leftMotor.stopMotor();
  }

  public Command runIntakeCommand() {
    return run(this::handleIntaking);
  }

  public Command stopIntakeCommand() {
    return run(this::stop);
  }

  public Command runOuttakeCommand() {
    return run(this::handleOuttaking);
  }
}
