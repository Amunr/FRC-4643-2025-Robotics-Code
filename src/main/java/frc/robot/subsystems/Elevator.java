package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.CANifier.LEDChannel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.config.*;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.elevatorConstants;

public class Elevator extends SubsystemBase  {
    public static SparkMax m_leftElevatorMotor = new SparkMax(Constants.elevatorConstants.leftElevatorCAN, MotorType.kBrushless);
    public static SparkMax m_rightElevatorMotor = new SparkMax(Constants.elevatorConstants.rightElevatorCAN, MotorType.kBrushless);
    SparkMaxConfig leftElevatorMotorConfig = new SparkMaxConfig();
    SparkMaxConfig rightElevatorMotorConfig = new SparkMaxConfig();
    static SparkClosedLoopController elevatorPID = m_leftElevatorMotor.getClosedLoopController();
    public RelativeEncoder leftElevatorEnc = m_leftElevatorMotor.getEncoder(); 
    double level = 0;
//    private SparkLimitSwitch limitSwitch = m_leftElevatorMotor.getReverseLimitSwitch(); 
        //   public elevatorSys m_elevatorSys;
     public Elevator(boolean PID){
        // this.m_elevatorSys = m_elevatorSys;
        if(PID){
        leftElevatorMotorConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake) ;
        leftElevatorMotorConfig.encoder
        .positionConversionFactor(8192 )
        .velocityConversionFactor(1);
        leftElevatorMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.00001, 0, 0);
      //   leftElevatorMotorConfig.limitSwitch
      //   .reverseLimitSwitchType(Type.kNormallyOpen)
      //   .reverseLimitSwitchEnabled(true);
     m_leftElevatorMotor.configure(leftElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
     rightElevatorMotorConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .follow(Constants.elevatorConstants.leftElevatorCAN);

        m_rightElevatorMotor.configure(rightElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 
        } else {
         leftElevatorMotorConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake) ;
        leftElevatorMotorConfig.encoder
        .positionConversionFactor(8192 )
        .velocityConversionFactor(1);
      //   leftElevatorMotorConfig.limitSwitch
      //   .reverseLimitSwitchType(Type.kNormallyOpen)
      //   .reverseLimitSwitchEnabled(true);
        m_leftElevatorMotor.configure(leftElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
     rightElevatorMotorConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .follow(Constants.elevatorConstants.leftElevatorCAN);
        m_rightElevatorMotor.configure(rightElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 


        }

     }
   //  public BooleanSupplier limitSwitchPressedSup = () -> (limitSwitch.isPressed());
     //positino is around 18.4,  do 10
     public void movePOS (){
    
       elevatorPID.setReference(level,SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0,3, ArbFFUnits.kVoltage);
    }
    public void setL1 (){
      level = elevatorConstants.level1Rotations; 

    }
    public void setL2 (){
      level = elevatorConstants.level2Rotations; 
    }
    public void setL3 (){
      level = elevatorConstants.level3Rotations; 
    }
    public void setL4 (){
      level = elevatorConstants.level4Rotations; 
    }
    public void setIntake () {
      level = elevatorConstants.levelIntakeRotations; 
    }

    
    public void resetEnc (){
       leftElevatorEnc.setPosition(0);
}
   public double getEnc (){
      return leftElevatorEnc.getPosition();
   }

public void manualControl (double speed, boolean enabled){
   if(enabled){
   if(speed>0.05 || speed <-0.05){
   m_rightElevatorMotor.set(-speed);
   m_leftElevatorMotor.set(speed);
   } else {
      m_rightElevatorMotor.setVoltage(3);
      m_leftElevatorMotor.setVoltage(3);
   }
}
}
}
