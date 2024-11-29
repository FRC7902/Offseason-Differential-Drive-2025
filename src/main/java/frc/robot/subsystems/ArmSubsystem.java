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
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

    //private final drive subsystem m_driveSubsystem;
    public final static ArmUtils util = new ArmUtils();
    private double m_setpoint = 0;

    //declare motor controller
    private final WPI_TalonSRX m_armLeaderMotor = new WPI_TalonSRX(ArmConstants.ArmLeaderMotorCAN);

    //object of simulated arm (used same numbers as CNE robot, not sure what to put)
    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
        DCMotor.getCIM(2),
        4096, //gear ration (how much input for output)
        0.0035, //moment of inirtia (determining property of load)
        0.639, //arm length
        0, //minimum angle arm reaches
        Math.PI, //maximum angle arm reaches?
        true, //gravity affects arm
        0); //starting arm angle

    //motor simulation
    private final TalonSRXSimCollection m_armLeaderMotorSim = m_armLeaderMotor.getSimCollection();

    //create new armsubsystem
    public ArmSubsystem() {
        configureMotors();
        configurePID();
    }

    //configure motors
    private void configureMotors() {
        //leader motor
        m_armLeaderMotor.configFactoryDefault();
        //motor settings
        m_armLeaderMotor.configContinuousCurrentLimit(10); //random # for now
        m_armLeaderMotor.configPeakCurrentLimit(20); //random # for now
    }

    //configure PID (regulate temperature/speed/flow/etc.)
    private void configurePID() {
        m_armLeaderMotor.config_kP(0, ArmConstants.kP);
        m_armLeaderMotor.config_kI(0, ArmConstants.kI);
        m_armLeaderMotor.config_kD(0, ArmConstants.kD);
    }

    public void setSetpoint(double Setpoint) {
        m_setpoint = Setpoint;
        m_armLeaderMotor.set(ControlMode.MotionMagic, util.degToCTRESensorUnits(Setpoint, ArmConstants.EncoderCPR));
    }

    public boolean atSetpoint() {
        return Math.abs(util.CTRESensorUnitsToDeg(getAngle(), ArmConstants.EncoderCPR) - m_setpoint) < ArmConstants.PositionTolerance;
    }

    @Override
    public void periodic() {
        //run commands for periodic cycle
        //display data to shuffleboard
        SmartDashboard.putNumber("Arm Angle", util.CTRESensorUnitsToDeg(getAngle(), ArmConstants.EncoderCPR));

        SmartDashboard.putNumber("Encoder Ticks", getAngle());
        SmartDashboard.putBoolean("At Setpoint", atSetpoint());
    }

    public double getAngle() { //?
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getAngle'");
    }

    public void stopMotor() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stopMotor'");
    }
}
