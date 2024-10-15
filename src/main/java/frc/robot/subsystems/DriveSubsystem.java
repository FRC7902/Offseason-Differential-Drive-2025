// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  private final CANSparkMax m_leftLeaderMotor = new CANSparkMax(33, MotorType.kBrushless);
  private final CANSparkMax m_leftFollowerMotor = new CANSparkMax(34, MotorType.kBrushless);
  private final CANSparkMax m_rightLeaderMotor = new CANSparkMax(31, MotorType.kBrushless);
  private final CANSparkMax m_rightFollowerMotor = new CANSparkMax(32, MotorType.kBrushless);

  public final DifferentialDrive m_drive = new DifferentialDrive(m_leftLeaderMotor, m_rightLeaderMotor);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_leftLeaderMotor.restoreFactoryDefaults();
    m_leftFollowerMotor.restoreFactoryDefaults();
    m_rightLeaderMotor.restoreFactoryDefaults();
    m_rightFollowerMotor.restoreFactoryDefaults();

    m_leftFollowerMotor.follow(m_leftLeaderMotor);
    m_rightFollowerMotor.follow(m_rightLeaderMotor);

    m_leftLeaderMotor.setInverted(true);

    m_leftLeaderMotor.setSmartCurrentLimit(45);
    m_leftFollowerMotor.setSmartCurrentLimit(45);
    m_rightLeaderMotor.setSmartCurrentLimit(45);
    m_rightFollowerMotor.setSmartCurrentLimit(45);

    m_leftLeaderMotor.setIdleMode(IdleMode.kBrake);
    m_leftFollowerMotor.setIdleMode(IdleMode.kBrake);
    m_rightLeaderMotor.setIdleMode(IdleMode.kBrake);
    m_rightFollowerMotor.setIdleMode(IdleMode.kBrake);
  }

  public void arcadeDrive(double forward, double rotation) {
    m_drive.arcadeDrive(forward, rotation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
