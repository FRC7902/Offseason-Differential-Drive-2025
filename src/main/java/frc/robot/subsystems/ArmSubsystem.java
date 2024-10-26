package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import com.revrobotics.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class ArmSubsystem extends SubsystemBase{
   private final WPI_TalonSRX m_armLeaderMotor = new WPI_TalonSRX (ArmConstants.ArmLeaderMotorCAN);
}

