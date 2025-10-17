package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Ports;

public class Arm extends SubsystemBase {
    private final SparkMax m_leadMotor = new SparkMax(Ports.CAN.ArmLead, MotorType.kBrushless);
    private final SparkMaxConfig m_leadConfig = new SparkMaxConfig();
    private final PIDController m_leadPID = new PIDController(0.5, 0, 0);

    private final SparkMax m_followerMotor = new SparkMax(Ports.CAN.ArmFollower, MotorType.kBrushless);
    private final SparkMaxConfig m_followerConfig = new SparkMaxConfig();
    private final PIDController m_followerPID = new PIDController(0.5, 0, 0);


    private final AbsoluteEncoder absEncoder = m_followerMotor.getAbsoluteEncoder(); // TODO factor and offset encoder for apporiate response of degrees for the PID calculate

    private double m_setpoint = absEncoder.getPosition();

    public Arm(){
        m_leadConfig.inverted(false).idleMode(IdleMode.kBrake);

        m_followerConfig.inverted(true).idleMode(IdleMode.kBrake);

        m_leadMotor.configure(m_leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_followerMotor.configure(m_followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setDesiredPosistion(double angle){
        m_setpoint = angle;
    }

    @Override
    public void periodic() {
       m_leadMotor.setVoltage(m_leadPID.calculate(absEncoder.getPosition(), m_setpoint)); // TODO apply ratio to setpoint
       m_followerMotor.setVoltage(m_followerPID.calculate(absEncoder.getPosition(), m_setpoint)); // TODO apply ratio setpoint
    }


}
