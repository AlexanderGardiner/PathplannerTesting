package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;



public class SwerveModule {
    // Define turn motor (SparkMax)
    CANSparkMax turnMotor;
    // Define drive motor (TalonFX)
     TalonFX driveMotor;
     final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
 
    public SwerveModule(int turnCANID, int driveCANID) {
        // Instantiate turn motor
        turnMotor = new CANSparkMax(turnCANID, MotorType.kBrushless);
        // Instantiate drive motor
        driveMotor  = new TalonFX(driveCANID);
        turnMotor.restoreFactoryDefaults();
        // Setup turn motor (with absolute encoder and PID wrapping)
        turnMotor.setInverted(true);
        turnMotor.getAbsoluteEncoder().setInverted(false);
        turnMotor.getPIDController().setPositionPIDWrappingEnabled(false);
        turnMotor.getPIDController().setPositionPIDWrappingMinInput(0);
        turnMotor.getPIDController().setPositionPIDWrappingMaxInput(2*Math.PI);
        // Setup drive motor
        driveMotor.setInverted(false);
        // Config PID loop for the turn motor
        turnMotor.getPIDController().setP(5);
        turnMotor.getPIDController().setI(0);
        turnMotor.getPIDController().setD(0);
        // Config PIDF loop for the drive motor
        
        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0;
        slot0Configs.kV = 0.5;
        slot0Configs.kP = 0;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
        driveMotor.getConfigurator().apply(slot0Configs);
    }



    public void setState(SwerveModuleState swerveModuleState) {
        // Set the target angle for the turn motor
        turnMotor.getPIDController().setReference(swerveModuleState.angle.getRadians(), ControlType.kPosition);
        // Set the target speed for the drive motor
        driveMotor.setControl((m_request.withVelocity(swerveModuleState.speedMetersPerSecond/(0.0762 * Math.PI)*60)));
    }

    public SwerveModulePosition getPos(){
        return new SwerveModulePosition((driveMotor.getPosition().getValue().baseUnitMagnitude()) * (0.0762 * Math.PI),new Rotation2d(turnMotor.getAbsoluteEncoder().getPosition()));
    }

}

