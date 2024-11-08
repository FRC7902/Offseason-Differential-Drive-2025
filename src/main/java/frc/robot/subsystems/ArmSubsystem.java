package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ArmUtils;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.sim.PhysicsSim;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class ArmSubsystem extends SubsystemBase {
  private double m_setpoint = 0;
  public final static ArmUtils util = new ArmUtils();

  private final WPI_TalonSRX m_armLeaderMotor = new WPI_TalonSRX(ArmConstants.kArmLeaderCAN);
  private final WPI_VictorSPX m_armFollowerMotor = new WPI_VictorSPX(ArmConstants.kArmFollowerCAN);

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
    m_armLeaderMotor.configFactoryDefault();
    m_armFollowerMotor.configFactoryDefault();

    // Only leader - config encoder
    m_armLeaderMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    m_armLeaderMotor.setSensorPhase(ArmConstants.sensorPhase);
    m_armLeaderMotor.setInverted(ArmConstants.motorInverted);
    m_armLeaderMotor.configAllowableClosedloopError(0, ArmConstants.kPositionTolerance, 0);
    m_armLeaderMotor.configVoltageCompSaturation(12, 0);

    m_armLeaderMotor.config_kP(0, ArmConstants.kP);
    m_armLeaderMotor.config_kI(0, ArmConstants.kI);
    m_armLeaderMotor.config_kD(0, ArmConstants.kD);

    m_armLeaderMotor.configMotionCruiseVelocity(ArmConstants.defaultSpeed);
    m_armLeaderMotor.configMotionAcceleration(ArmConstants.defaultAcceleration);


    m_armFollowerMotor.follow(m_armLeaderMotor);

    PhysicsSim.getInstance().addTalonSRX(m_armLeaderMotor, 0.75, 4000, ArmConstants.sensorPhase);
    PhysicsSim.getInstance().addVictorSPX(m_armFollowerMotor);

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Angle", util.CTRESensorUnitsToDeg(getAngle()));

    SmartDashboard.putNumber("Encoder Ticks", getAngle());
    SmartDashboard.putBoolean("At Setpoint", atSetpoint());

  }

  @Override
  public void simulationPeriodic(){
    PhysicsSim.getInstance().run();
  }

  public double getAngle() {
    // Get raw angle of arm in ticks (4096 per revolution)
    return m_armLeaderMotor.getSensorCollection().getQuadraturePosition();
  }

  public void setAngle(double angleInDegrees){
    
    m_armLeaderMotor.set(ControlMode.MotionMagic, util.degToCTRESensorUnits(angleInDegrees));

  }

  public boolean atSetpoint() {
    return Math.abs(
        util.CTRESensorUnitsToDeg(getAngle()) - m_setpoint) < ArmConstants.kPositionTolerance;
  }

}
