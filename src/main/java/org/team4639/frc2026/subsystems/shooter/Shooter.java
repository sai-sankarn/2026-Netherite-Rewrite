package org.team4639.frc2026.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team4639.frc2026.util.PortConfiguration;

public class Shooter extends SubsystemBase {
  //   private final SparkFlex leftMotor;
  private final SparkFlex rightMotor;

  private double volts = 4;

  public Shooter(PortConfiguration ports) {
    // leftMotor = new SparkFlex(ports.shooterMotorLeftID.getDeviceNumber(), MotorType.kBrushless);
    rightMotor = new SparkFlex(30, MotorType.kBrushless);

    SparkFlexConfig config = new SparkFlexConfig();

    config
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(5)
        .appliedOutputPeriodMs(5)
        .busVoltagePeriodMs(5)
        .outputCurrentPeriodMs(5);
    rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // config.follow(rightMotor, true);

    // leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putNumber("Shooter Volts", volts);
  }

  public void periodic() {
    volts = SmartDashboard.getNumber("Shooter Volts", 0);
  }

  public Command runShooterCommand() {

    return run(
        () -> {
          rightMotor.setVoltage(volts);
        });
  }

  public Command stopShooterCommand() {
    return run(() -> rightMotor.stopMotor());
  }
}
