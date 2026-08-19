package org.team4639.frc2026.subsystems.shooter;

import org.team4639.frc2026.util.PortConfiguration;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    private final SparkFlex leftMotor;
    private final SparkFlex rightMotor;

    private double volts;

    public Shooter(PortConfiguration ports){
        leftMotor = new SparkFlex(ports.shooterMotorLeftID.getDeviceNumber(), MotorType.kBrushless);
        rightMotor = new SparkFlex(ports.shooterMotorRightID.getDeviceNumber(), MotorType.kBrushless);

        SparkBaseConfig config = new SparkFlexConfig();
        config.follow(ports.shooterMotorRightID.getDeviceNumber(),true);

        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        volts = 0;
    }

    public void periodic(){
        volts = SmartDashboard.getNumber("Shooter Volts", 0);
    }

    public Command runShooterCommand(){
        return run(()-> rightMotor.setVoltage(volts));
    }

    public Command stopShooterCommand(){
        return run(()-> rightMotor.stopMotor());
    }



}
