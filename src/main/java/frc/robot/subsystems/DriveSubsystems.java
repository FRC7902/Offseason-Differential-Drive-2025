// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//Change -1s in class to device IDs
public class DriveSubsystems extends SubsystemBase {
  private final CANSparkMax m_leftLeaderMotor = new CANSparkMax(-1, MotorType.kBrushless);
  private final CANSparkMax m_leftFollowerMotor = new CANSparkMax(-1, MotorType.kBrushless);
  private final CANSparkMax m_rightLeaderMotor = new CANSparkMax(-1, MotorType.kBrushless);
  private final CANSparkMax m_rightFollowerMotor = new CANSparkMax(-1, MotorType.kBrushless);
  public final DifferentialDrive m_Drive;
  
  /** Creates a new DriveSubsystems. */
     public DriveSubsystems() {
      m_leftLeaderMotor.restoreFactoryDefaults();
      m_leftFollowerMotor.restoreFactoryDefaults();
      m_rightFollowerMotor.restoreFactoryDefaults();
      m_rightLeaderMotor.restoreFactoryDefaults();
      m_leftFollowerMotor.follow(m_leftLeaderMotor);
      m_rightFollowerMotor.follow(m_rightLeaderMotor);
      m_rightLeaderMotor.setSmartCurrentLimit(45);
      m_rightFollowerMotor.setSmartCurrentLimit(45);
      m_leftFollowerMotor.setSmartCurrentLimit(45);
      m_leftLeaderMotor.setSmartCurrentLimit(45);
      m_leftLeaderMotor.setIdleMode(IdleMode.kBrake);
      m_leftFollowerMotor.setIdleMode(IdleMode.kBrake);
      m_rightLeaderMotor.setIdleMode(IdleMode.kBrake);
      m_rightFollowerMotor.setIdleMode(IdleMode.kBrake);
      m_Drive = new DifferentialDrive(m_leftLeaderMotor, m_rightLeaderMotor);
     }
  public void arcadeDrive( double speed, double rotation ) {
    m_Drive.arcadeDrive(speed, rotation);
    

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
