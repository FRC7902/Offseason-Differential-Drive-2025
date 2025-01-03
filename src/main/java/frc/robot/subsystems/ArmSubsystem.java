// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ArmUtils;
import frc.robot.Constants.OperatorConstants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private final WPI_TalonSRX m_armLeaderMotor = new WPI_TalonSRX(ArmConstants.kArmMotorCAN);
  public final static ArmUtils util = new ArmUtils();
  private double m_setpoint = 0;
  private final SingleJointedArmSim armSim = new SingleJointedArmSim(
 DCMotor.getCIM(2),
  	139.78,
  	0.0035, // Moment of intertia
  	0.639, // 0.3193m*2
  	0,
  	Math.PI,
  	true,
  	0);
  private final TalonSRXSimCollection m_armLeaderMotorSim = new TalonSRXSimCollection(m_armLeaderMotor);



  public double getAngle() {
    // Get raw angle of arm in ticks (4096 per revolution)
    return m_armLeaderMotor.getSensorCollection().getQuadraturePosition();
    }
  public boolean atSetpoint() {
    return Math.abs(
        ArmUtils.CTRESensorUnitsToDeg(getAngle()) - m_setpoint) < ArmConstants.kPositionTolerance;
    }

    
public void setSetpoint(double setpoint) {
  m_setpoint = setpoint;
  m_armLeaderMotor.set(ControlMode.MotionMagic, util.degToCTRESensorUnits(ArmConstants.EncoderCPR));
}

  public ArmSubsystem() {

    }

    @Override
    public void periodic() {
      SmartDashboard.putNumber("Arm Angle", ArmUtils.CTRESensorUnitsToDeg(getAngle()));
  
      SmartDashboard.putNumber("Encoder Ticks", getAngle());
      SmartDashboard.putBoolean("At Setpoint", atSetpoint());
    }
}
