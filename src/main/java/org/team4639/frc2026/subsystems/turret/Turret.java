package org.team4639.frc2026.subsystems.turret;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.util.LimelightHelpers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class Turret extends SubsystemBase {
  private final TalonFX motor;

  private final PIDController turretPIDController= new PIDController(0, 0, 0); 
  private final String TURRET_LIMELIGHT_NAME = ""; 
  private static double kP = 0.03; 
  private final VoltageOut turretVoltageControl = new VoltageOut(0); 
  
  public Turret(PortConfiguration ports) {
    motor = new TalonFX(ports.TurretMotorID.getDeviceNumber(), "2026CANivore");
    turretPIDController.setSetpoint(0);
    turretPIDController.setTolerance(0.5);
    turretPIDController.setD(kP); 
    SmartDashboard.putNumber("Turret Motor kP", kP); 
  }

  public void periodic() {
    kP = SmartDashboard.getNumber("Turret Motor kP", 0);
    turretPIDController.setD(kP); 
  }

  public Command moveLeft() {
    return run(() -> motor.setVoltage(3));
  } 

  public Command moveRight() {
    return run(() -> motor.setVoltage(-3));
  }

  public Command centerAprilTag() {
    return run(() -> {
      boolean hasTarget = LimelightHelpers.getTV(TURRET_LIMELIGHT_NAME); 
      if (hasTarget) {
        double tx = LimelightHelpers.getTX(TURRET_LIMELIGHT_NAME); 
        
        double pidOutput = turretPIDController.calculate(tx, 0.0); 
        pidOutput = Math.max(-10.0, Math.min(10.0, pidOutput));
        motor.setControl(turretVoltageControl.withOutput(pidOutput)); 
      }
    }); 
  }
}
