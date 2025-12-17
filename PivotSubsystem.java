package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
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
import com.revrobotics.spark.SparkBase.ResetMode;

public class PivotSubsystem extends SubsystemBase {

  private DutyCycleEncoder m_absoluteEncoder;
  private RelativeEncoder m_relativeEncoder;

  private SparkMax m_motor;
  private SparkClosedLoopController m_controller;
  private SparkMaxConfig config;

  public PivotSubsystem()
  {
  
    m_motor = new SparkMax(ArmConstants.MOTOR_ID, MotorType.kBrushless);
    m_absoluteEncoder = new DutyCycleEncoder(ArmConstants.ENCODER_PORT, 360, ArmConstants.ENCODER_OFFSET);
    m_relativeEncoder = m_motor.getEncoder();
    
    // initialize PID controller
    m_controller = m_motor.getClosedLoopController();
    
    config = new SparkMaxConfig();

    // configure PID
    config.closedLoop
    .pid(ArmConstants.Kp, ArmConstants.Ki, ArmConstants.Kd, ClosedLoopSlot.kSlot1)
    .outputRange(-ArmConstants.SPEED_LIMIT, ArmConstants.SPEED_LIMIT)
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .maxMotion
    .allowedClosedLoopError(0.5);

    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
    // reset encoder offset
    resetRelativeToAbsolute();
  }

  /*
   * sets the motor to a given angle
   */
  public void SetPosition(double angle) 
  {
    m_controller.setReference(angle, ControlType.kPosition);
  }
  
  /*
   * returnes the angle of the relative encoder
   */
  public double getAngle() {
    return m_relativeEncoder.getPosition();
  }

  /*
   * returns the position of the absolute encoder
   */
  public double getAbsoluteAngle() {
    return m_absoluteEncoder.get();
  }

  /*
   * resets the relative encoder based on the absolute encoder's reading
   */
  private void resetRelativeToAbsolute() {
    double absoluteDeg = getAbsoluteAngle();
    m_relativeEncoder.setPosition(absoluteDeg);
  }
}