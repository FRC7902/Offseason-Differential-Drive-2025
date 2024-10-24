// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

public class DriveSubsystem extends SubsystemBase {

  public static CANSparkMax leftCanSparkMax_1 = (null);
  public static CANSparkMax leftCanSparkMax_2 = (null);
  public static CANSparkMax rightCanSparkMax_1 = (null);
  public static CANSparkMax rightCanSparkMax_2 = (null);
  DifferentialDrive differentialDrive = null;



  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    leftCanSparkMax_1.restoreFactoryDefaults();
    leftCanSparkMax_2.restoreFactoryDefaults();
    rightCanSparkMax_1.restoreFactoryDefaults();
    rightCanSparkMax_2.restoreFactoryDefaults();
    leftCanSparkMax_1 = new CANSparkMax(OperatorConstants.DriveSubsystem_Left_CANSparkMax_1,MotorType.kBrushless);
    leftCanSparkMax_2 = new CANSparkMax(OperatorConstants.DriveSubsystem_Left_CANSparkMax_2,MotorType.kBrushless);
    rightCanSparkMax_1 = new CANSparkMax(OperatorConstants.DriveSubsystem_Right_CANSparkMax_1,MotorType.kBrushless);
    rightCanSparkMax_2 = new CANSparkMax(OperatorConstants.DriveSubsystem_Right_CANSparkMax_2,MotorType.kBrushless);
    leftCanSparkMax_2.follow(leftCanSparkMax_1);
    rightCanSparkMax_2.follow(rightCanSparkMax_1);
    differentialDrive = new DifferentialDrive(leftCanSparkMax_1, rightCanSparkMax_1);
  }

  public void  tankDrive(double leftSpeed, double rightSpeed)  {
    differentialDrive.tankDrive(leftSpeed, rightSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
