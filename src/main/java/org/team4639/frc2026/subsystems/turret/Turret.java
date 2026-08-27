package org.team4639.frc2026.subsystems.turret;

import org.team4639.frc2026.util.PortConfiguration;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase{
    private final TalonFX motor;

    public Turret(PortConfiguration ports){
        motor = new TalonFX(ports.TurretMotorID.getDeviceNumber(), "2026CANivore");
    }

    public Command moveLeft(){
        return run(()-> motor.setVoltage(3));
    }

    public Command moveRight(){
        return run(() -> motor.setVoltage(-3));
    }

}
