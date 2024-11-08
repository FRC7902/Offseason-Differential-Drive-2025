package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ArmUtils;
import frc.robot.Constants.ArmConstants;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class ArmSubsystem extends SubsystemBase {
    private double m_setpoint = 0;
     public final static ArmUtils util = new ArmUtils();
    private final WPI_TalonSRX m_armLeaderMotor = new WPI_TalonSRX(ArmConstants.kArmMotorCAN);
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
        // configureMotors();
        // configurePID();
    }

    @Override
    public void periodic() {
  SmartDashboard.putNumber("Arm Angle", util.CTRESensorUnitsToDeg(getAngle()));

  SmartDashboard.putNumber("Encoder Ticks", getAngle());
  SmartDashboard.putBoolean("At Setpoint", atSetpoint());
}

public double getAngle() {
    // Get raw angle of arm in ticks (4096 per revolution)
    return m_armLeaderMotor.getSensorCollection().getQuadraturePosition();
  }
  public boolean atSetpoint() {
    return Math.abs(
        util.CTRESensorUnitsToDeg(getAngle()) - m_setpoint) < ArmConstants.kPositionTolerance;
  }


}
