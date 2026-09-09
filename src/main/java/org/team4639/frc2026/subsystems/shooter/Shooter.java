package org.team4639.frc2026.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.util.LoggedTunableNumber;

public class Shooter extends SubsystemBase {
  private final SparkFlex leftMotor;
  private final SparkFlex rightMotor;

  private final SparkFlexConfig config = new SparkFlexConfig();

  private final SysIdRoutine sysIdRoutine;

  private LoggedTunableNumber kP = new LoggedTunableNumber("Shooter kP");

  private LoggedTunableNumber kS = new LoggedTunableNumber("Shooter kS");
  private LoggedTunableNumber kV = new LoggedTunableNumber("Shooter kV");
  private LoggedTunableNumber kA = new LoggedTunableNumber("Shooter kA");

  private LoggedTunableNumber setpoint = new LoggedTunableNumber("Shooter Setpoint");

  private double volts = 4;

  public Shooter(PortConfiguration ports) {
    leftMotor = new SparkFlex(ports.shooterMotorLeftID.getDeviceNumber(), MotorType.kBrushless);
    rightMotor = new SparkFlex(30, MotorType.kBrushless);

    config
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(5)
        .appliedOutputPeriodMs(5)
        .busVoltagePeriodMs(5)
        .outputCurrentPeriodMs(5);
    config.closedLoop.p(kP.getAsDouble());
    config.closedLoop.feedForward.sva(kS.getAsDouble(), kV.getAsDouble(), kA.getAsDouble());
    rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(rightMotor, true);

    leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putNumber("Shooter Volts", volts);

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(1), // Ramp rate (1V/s)
                Volts.of(7), // Step voltage (7V dynamic test limit)
                null // Timeout (default 10s)
                ),
            new SysIdRoutine.Mechanism(
                (Voltage voltage) -> rightMotor.setVoltage(voltage.in(Volts)),
                log -> {
                  log.motor("shooter")
                      .voltage(Volts.of(rightMotor.getAppliedOutput() * rightMotor.getBusVoltage()))
                      .angularPosition(Rotations.of(rightMotor.getEncoder().getPosition()))
                      .angularVelocity(
                          Rotations.per(Second).of(rightMotor.getEncoder().getVelocity()));
                  Logger.recordOutput(getName(), 0);
                },
                this));

    // setDefaultCommand(Commands.runOnce(() -> rightMotor.setVoltage(0), this));
    kP.initDefault(0);
    kS.initDefault(0);
    kV.initDefault(0);
    kA.initDefault(0);
    setpoint.initDefault(0);
  }

  public void periodic() {
    volts = SmartDashboard.getNumber("Shooter Volts", 0);
    config.closedLoop.feedForward.sva(kS.getAsDouble(), kV.getAsDouble(), kA.getAsDouble());
    config.closedLoop.p(kP.getAsDouble());
  }

  // public Command runShooterCommand() {

  //   return run(
  //       () -> {
  //         rightMotor.setVoltage(volts);
  //       });
  // }

  public Command runShooterCommand() {

    return run(
        () -> {
          rightMotor
              .getClosedLoopController()
              .setSetpoint(setpoint.getAsDouble(), ControlType.kVelocity);
        });
  }

  public Command stopShooterCommand() {
    return run(() -> rightMotor.stopMotor());
  }

  public Command forwardQuasistaticCommand() {
    return sysIdRoutine.quasistatic(Direction.kForward);
  }

  public Command backwardQuasistaticCommand() {
    return sysIdRoutine.quasistatic(Direction.kReverse);
  }

  public Command forwardDynamicCommand() {
    return sysIdRoutine.dynamic(Direction.kForward);
  }

  public Command backwardDynamicCommand() {
    return sysIdRoutine.dynamic(Direction.kReverse);
  }
}
