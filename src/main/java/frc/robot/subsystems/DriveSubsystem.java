// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class DriveSubsystem extends SubsystemBase {

  private final CANSparkMax m_leftLeaderMotor = new CANSparkMax(33, MotorType.kBrushless);
  private final CANSparkMax m_leftFollowerMotor = new CANSparkMax(34, MotorType.kBrushless);
  private final CANSparkMax m_rightLeaderMotor = new CANSparkMax(31, MotorType.kBrushless);
  private final CANSparkMax m_rightFollowerMotor = new CANSparkMax(32, MotorType.kBrushless);

  //dummy encoder declaration
  private final Encoder m_leftDummyEncoder = new Encoder (12,13);
  private final Encoder m_rightDummyEncoder = new Encoder (14,15);
  private final AnalogGyro m_DummyGyro = new AnalogGyro(1);

  //simulation of encoders, gyro, and drivetrain
  private final EncoderSim m_rightEncoderSim;
  private final EncoderSim m_leftEncoderSim;
  private final AnalogGyroSim m_gyroSim;
  private final DifferentialDrivetrainSim m_driveTrainSim;

  //simulation of field
  private final Field2d m_Field2d;
  
  //odometry (not specifically for simulation, use data to estimate change in position)
  private final DifferentialDriveOdometry m_odometry;

  public final DifferentialDrive m_drive = new DifferentialDrive(m_leftLeaderMotor, m_rightLeaderMotor);

  private final RelativeEncoder m_leftEncoder = m_leftLeaderMotor.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightLeaderMotor.getEncoder();
  
    /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    //connect simulated devices to physical/dummy devices
    m_leftEncoderSim = new EncoderSim(m_leftDummyEncoder);
    m_rightEncoderSim = new EncoderSim(m_rightDummyEncoder);
    m_gyroSim = new AnalogGyroSim(m_DummyGyro);

    //define physical parameters of simulated drivetrain
    m_driveTrainSim = DifferentialDrivetrainSim.createKitbotSim(KitbotMotor.kDoubleNEOPerSide,KitbotGearing.k10p71,KitbotWheelSize.kSixInch,null);

    //simulate field and put on dashboard
    m_Field2d = new Field2d();
    SmartDashboard.putData(m_Field2d);

    //decide to use simulated devices or not depending on simulation or not
    if (Robot.isSimulation()) {
      m_odometry = new DifferentialDriveOdometry(
        m_DummyGyro.getRotation2d(),
        m_leftDummyEncoder.getDistance(),
        m_rightDummyEncoder.getDistance(),
        new Pose2d(1,1,new Rotation2d()));
    } else {
      m_odometry = new DifferentialDriveOdometry(
        m_DummyGyro.getRotation2d(),
        m_leftEncoder.getPosition(),
        m_rightEncoder.getPosition(),
        new Pose2d(1,1,new Rotation2d()));
    }

    m_leftFollowerMotor.restoreFactoryDefaults();
    m_leftLeaderMotor.restoreFactoryDefaults();
    m_rightFollowerMotor.restoreFactoryDefaults();
    m_rightLeaderMotor.restoreFactoryDefaults();

    //set followers and leaders
    m_leftFollowerMotor.follow(m_leftLeaderMotor);
    m_rightFollowerMotor.follow(m_rightLeaderMotor);

    //invert motor
    m_leftLeaderMotor.setInverted(true);

    //set max motor limit
    m_leftLeaderMotor.setSmartCurrentLimit(45);
    m_leftFollowerMotor.setSmartCurrentLimit(45);
    m_rightLeaderMotor.setSmartCurrentLimit(45);
    m_rightFollowerMotor.setSmartCurrentLimit(45);

    //set idle mode (when not in motion)
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
    //update odometry with dummy devices
    m_odometry.update(m_DummyGyro.getRotation2d(),m_leftDummyEncoder.getDistance(),m_rightDummyEncoder.getDistance());
    
    //update robot position on field
    m_Field2d.setRobotPose(m_odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {
    //translate motor output [-1,1] to voltage for simulation (units)
    m_driveTrainSim.setInputs(m_leftLeaderMotor.get() * RobotController.getInputVoltage(),m_rightLeaderMotor.get() * RobotController.getInputVoltage());

    //set update delay like the real 20 ms delay for NON-SIMULATED robots
    m_driveTrainSim.update(0.02);

    //update all sensors
    //dummy devices must be updated based on the real devices
    m_leftEncoderSim.setDistance(m_driveTrainSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_driveTrainSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_driveTrainSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_driveTrainSim.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(m_driveTrainSim.getHeading().getDegrees());
  }

}
