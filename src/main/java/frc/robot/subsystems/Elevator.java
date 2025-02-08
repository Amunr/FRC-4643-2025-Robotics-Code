package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.config.*;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase  {
    public static SparkMax m_leftElevatorMotor = new SparkMax(1, MotorType.kBrushless);
    SparkMaxConfig elevatorMotorConfig = new SparkMaxConfig();
    static SparkClosedLoopController elevatorPID = m_leftElevatorMotor.getClosedLoopController();
    //   public elevatorSys m_elevatorSys;
 public Elevator(){
    // this.m_elevatorSys = m_elevatorSys;
    
    elevatorMotorConfig
    .inverted(false)
    .idleMode(IdleMode.kBrake);
    elevatorMotorConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);
    elevatorMotorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(0.00001, 0.0, 0.0);
 m_leftElevatorMotor.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


 }

 //positino is around 18.4,  do 10
 public static void movePOS (double position){
    elevatorPID.setReference(position * 2048, SparkBase.ControlType.kPosition);

}

}
