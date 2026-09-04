package org.team4639.frc2026.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.util.LimelightHelpers;

public class Turret extends SubsystemBase {
  private final TalonFX motor;
  private boolean runningTurret = false;

  private final PIDController turretPIDController = new PIDController(0, 0, 0);
  private final String TURRET_LIMELIGHT_NAME = "limelight-turret";
  private static double kP = 0.03;
  private final VoltageOut turretVoltageControl = new VoltageOut(0);

  public Turret(PortConfiguration ports) {
    motor = new TalonFX(ports.TurretMotorID.getDeviceNumber(), "2026CANIvore");
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.CurrentLimits.SupplyCurrentLimit = 20.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 30;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    double kS = 0.1813163;
    double kV = 0.01055999667;
    double kA = 0.0007793003;

    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    config.Slot0.kP = 0.5; // mild kP, most of control here is done through velocity

    config.Slot1.kP = 15; // most aggressive for turret wraparound

    config.Slot2.kP =
        4; // want to reach zero setpoint with accuracy, don't care about speed but don't
    // want
    // steady state error

    motor.getConfigurator().apply(config);

    turretPIDController.setSetpoint(0);
    turretPIDController.setTolerance(0.5);
    turretPIDController.setP(kP);
    SmartDashboard.putNumber("Turret Motor kP", kP);
  }

  public Command moveLeft() {
    return run(() -> {
          motor.setVoltage(3);
          runningTurret = true;
        })
        .finallyDo(
            () -> {
              runningTurret = false;
              motor.stopMotor();
            });
  }

  public Command moveRight() {
    return run(() -> {
          motor.setVoltage(-3);
          runningTurret = true;
        })
        .finallyDo(
            () -> {
              runningTurret = false;
              motor.stopMotor();
            });
  }

  public Command centerAprilTag() {
    return run(
        () -> {
          boolean hasTarget = LimelightHelpers.getTV(TURRET_LIMELIGHT_NAME);
          if (hasTarget) {
            double tx = LimelightHelpers.getTX(TURRET_LIMELIGHT_NAME);

            double pidOutput = turretPIDController.calculate(tx, 0.0);
            pidOutput = Math.max(-10.0, Math.min(10.0, pidOutput));
            motor.setControl(turretVoltageControl.withOutput(pidOutput));
          } else {

          }
        });
  }

  public void periodic() {
    SmartDashboard.putNumber("Turret Applied Voltage", motor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putBoolean("Turret Moving", runningTurret);
    kP = SmartDashboard.getNumber("Turret Motor kP", 0);
    turretPIDController.setP(kP);
  }
}
