package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.CANifier.LEDChannel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.config.*;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.elevatorConstants;

public class Elevator extends SubsystemBase  {
    public static SparkMax m_leftElevatorMotor = new SparkMax(Constants.elevatorConstants.leftElevatorCAN, MotorType.kBrushless);
    public static SparkMax m_rightElevatorMotor = new SparkMax(Constants.elevatorConstants.rightElevatorCAN, MotorType.kBrushless);
    SparkMaxConfig leftElevatorMotorConfig = new SparkMaxConfig();
    SparkMaxConfig rightElevatorMotorConfig = new SparkMaxConfig();
    static SparkClosedLoopController elevatorPID = m_leftElevatorMotor.getClosedLoopController();
    private static RelativeEncoder leftElevatorEnc = m_leftElevatorMotor.getAlternateEncoder(); 
    double level = 0; 
        //   public elevatorSys m_elevatorSys;
     public Elevator(){
        // this.m_elevatorSys = m_elevatorSys;
        
        leftElevatorMotorConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake) ;
        leftElevatorMotorConfig.encoder
        .positionConversionFactor(8192 )
        .velocityConversionFactor(1);
        leftElevatorMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        .pid(0.00001, 0, 0);
     m_leftElevatorMotor.configure(leftElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
     rightElevatorMotorConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .follow(m_leftElevatorMotor);

        m_rightElevatorMotor.configure(rightElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
        DoubleSupplier leftElevatorEncPos = () -> leftElevatorEnc.getPosition();
     }
    
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

public void manualControl (double speed, boolean enabled){
      if(speed>0.05 || speed <-0.05){
   m_rightElevatorMotor.set(-speed);
   m_leftElevatorMotor.set(speed);
   } else {
      m_rightElevatorMotor.setVoltage(3);
      m_leftElevatorMotor.setVoltage(3);
   }
}

}
