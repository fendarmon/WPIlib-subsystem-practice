package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorPIDConstants;
import frc.robot.Constants.ElevatorFeedforwardConstants;;

public class ElevatorSubsystem extends SubsystemBase {

  private RelativeEncoder m_relativeEncoder;

  private SparkMax m_motor;
  private SparkClosedLoopController m_controller;
  private SparkMaxConfig config;

  // feedforward constants
  private static final double kS = ElevatorFeedforwardConstants.kS; // static
  private static final double kG = ElevatorFeedforwardConstants.kG; // gravity
  private static final double kV = ElevatorFeedforwardConstants.kV; // gravity
  private static final double kA = ElevatorFeedforwardConstants.kA; // acceleration

  private ElevatorFeedforward eff;

  public ElevatorSubsystem()
  {
  
    m_motor = new SparkMax(ElevatorConstants.ELEVATE_MOTOR_ID, MotorType.kBrushless);
    m_relativeEncoder = m_motor.getEncoder();
    
    // initialize PID controller & feedForward
    m_controller = m_motor.getClosedLoopController();
    eff = new ElevatorFeedforward(kS, kG, kA);
    
    config = new SparkMaxConfig();

    // configure PID
    config.closedLoop
    .velocityFF(1.0/kV)
    .pid(ElevatorPIDConstants.kP, ElevatorPIDConstants.kI, ElevatorPIDConstants.kD, ClosedLoopSlot.kSlot1)
    .outputRange(-1, 1)
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .maxMotion
    .allowedClosedLoopError(0.5);

    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /*
   * sets the motor to a given angle
   */
  public void SetPosition(double targetheight) 
  {
    double arbFFVolts = eff.calculate(Math.toRadians(targetheight));

    m_controller.setReference(targetheight, ControlType.kPosition, ClosedLoopSlot.kSlot0, arbFFVolts);
  }
  
  /*
   * sets the the motor with a given voltage
   */
  public void Set(double voltage) 
  {
    // returns +- 1 depends on the value
    double direction = Math.signum(voltage);
    // target velocity is 0 because we want gravity compensation
    double arbFFVolts = eff.calculate(0.0) * direction;

    m_controller.setReference(voltage, ControlType.kVoltage, ClosedLoopSlot.kSlot0, arbFFVolts);
  }
  
  /*
   * returns the angle of the relative encoder of the brushless motor
   */
  public double getCurrentHeight() {
    return m_relativeEncoder.getPosition();
  }
}