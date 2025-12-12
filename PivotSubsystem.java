package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class PivotSubsystem extends SubsystemBase {

  private DutyCycleEncoder m_absoluteEncoder;
  private RelativeEncoder m_relativeEncoder;

  private SparkMax m_motor;
  private SparkClosedLoopController m_controller;
  private SparkMaxConfig config;

  // feedforward constants
  private static final double kS = ArmConstants.Ks; // static
  private static final double kV = ArmConstants.Kv; // velocity
  private static final double kG = ArmConstants.Kg; // gravity
  private static final double kA = ArmConstants.Ka; // acceleration

  private ArmFeedforward aff;
  private double voltage;

  
  public PivotSubsystem()
  {
  
    m_motor = new SparkMax(ArmConstants.MOTOR_ID, MotorType.kBrushless);
    m_absoluteEncoder = new DutyCycleEncoder(ArmConstants.ENCODER_PORT, 360, ArmConstants.ENCODER_OFFSET);
    m_relativeEncoder = m_motor.getEncoder();
    
    // initialize PID controller & feedForward
    m_controller = m_motor.getClosedLoopController();
    aff = new ArmFeedforward(kS, kG, kV, kA);
    
    config = new SparkMaxConfig();

    // configure PID
    config.closedLoop
    .pid(Constants.ArmConstants.Kp, ArmConstants.Ki, ArmConstants.Kd, ClosedLoopSlot.kSlot1)
    .outputRange(-ArmConstants.SPEED_LIMIT, ArmConstants.SPEED_LIMIT)
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .velocityFF(kV, ClosedLoopSlot.kSlot0)
    .maxMotion
    .allowedClosedLoopError(0.5);

    m_motor.configure(config, null, null);
    
    // reset encoder offset
    resetRelativeToAbsolute();
  }

    public void SetPosition(double angle) 
  {
    m_controller.setReference(angle, ControlType.kPosition);
  }
  
  public double getAngle() {
    return m_relativeEncoder.getPosition();
  }

  public double getAbsoluteAngle() {
    return m_absoluteEncoder.get();
  }

  private void resetRelativeToAbsolute() {
    double absoluteDeg = getAbsoluteAngle();
    m_relativeEncoder.setPosition(absoluteDeg);
  }

  private double calculateFF() 
  {
    voltage = aff.calculate(Math.toRadians(getAngle()), /* help */);
    return voltage;
  }
}