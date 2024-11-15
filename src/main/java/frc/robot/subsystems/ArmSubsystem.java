package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants.ArmConstants;
import frc.robot.ArmUtils;


public class ArmSubsystem extends SubsystemBase {
    // Motor and arm simulation, arm simulation we will do later
    public final static ArmUtils util = new ArmUtils();
    private double m_setpoint = 0;
    private final WPI_TalonSRX m_armLeaderMotor = new WPI_TalonSRX(ArmConstants.ArmLeaderMotorCAN);
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

    public ArmSubsystem() {
        //configureMotors();
        // configurePID();
    }

    public double getAngle() {
        // Get raw angle of arm in ticks (4096 per revolution)
        return m_armLeaderMotor.getSensorCollection().getQuadraturePosition();
      }

    public void setSetpoint(double setpoint) {
        m_setpoint = setpoint;
        m_armLeaderMotor.set(ControlMode.MotionMagic, util.degToCTRESensorUnits(setpoint, ArmConstants.EncoderCPR));
    }
  
    public boolean atSetpoint() {
        return Math.abs(util.CTRESensorUnitsToDeg(getAngle(), ArmConstants.EncoderCPR) - m_setpoint) < ArmConstants.PositionTolerance;
    }
      
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Angle", util.CTRESensorUnitsToDeg(getAngle(), ArmConstants.EncoderCPR));

        SmartDashboard.putNumber("Encoder Ticks", getAngle());
        SmartDashboard.putBoolean("At Setpoint", atSetpoint());
    }
    
}

