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
import edu.wpi.first.math.controller.ElevatorFeedforward;
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
  private static final double kV = ElevatorFeedforwardConstants.kV; // velocity
  private static final double kG = ElevatorFeedforwardConstants.kG; // gravity
  private static final double kA = ElevatorFeedforwardConstants.kA; // acceleration

  private ElevatorFeedforward eff;
  private double voltage;

  public ElevatorSubsystem()
  {
  
    m_motor = new SparkMax(ElevatorConstants.ELEVATE_MOTOR_ID, MotorType.kBrushless);
    m_relativeEncoder = m_motor.getEncoder();
    
    // initialize PID controller & feedForward
    m_controller = m_motor.getClosedLoopController();
    eff = new ElevatorFeedforward(kS, kG, kV, kA);
    
    config = new SparkMaxConfig();

    // configure PID
    config.closedLoop
    .pid(ElevatorPIDConstants.kP, ElevatorPIDConstants.kI, ElevatorPIDConstants.kD, ClosedLoopSlot.kSlot1)
    .outputRange(-1, 1)
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .maxMotion
    .allowedClosedLoopError(0.5);

    m_motor.configure(config, null, null);
  }

  public void SetPosition(double angle) 
  {
    m_controller.setReference(angle, ControlType.kPosition);
  }

  public void Set(double voltage) 
  {
    m_controller.setReference(voltage, ControlType.kVoltage);
  }
  
  public double getAngle() {
    return m_relativeEncoder.getPosition();
  }

  // get feedforward voltage for the Set() method
  private double calculateFF(double targetSpeed) 
  {
    voltage = eff.calculate(Math.toRadians(getAngle()), targetSpeed);
    return voltage;
  }
}