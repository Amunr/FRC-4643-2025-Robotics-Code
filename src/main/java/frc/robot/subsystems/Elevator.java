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

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.CANifier.LEDChannel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.config.*;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    double level = Constants.elevatorConstants.level1Rotations;
//    private SparkLimitSwitch limitSwitch = m_leftElevatorMotor.getReverseLimitSwitch(); 
        //   public elevatorSys m_elevatorSys;
     public Elevator(){
      SmartDashboard.putString("Elevator Set Level", "level One");
        // this.m_elevatorSys = m_elevatorSys;
        leftElevatorMotorConfig
        .inverted(true)
        .idleMode(IdleMode.kCoast) ;
        leftElevatorMotorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);
        leftElevatorMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.065, 0, 0)
        .maxOutput(0.9)
        ;
      //   leftExlevatorMotorConfig.limitSwitch
      //   .reverseLimitSwitchType(Type.kNormallyOpen)
      //   .reverseLimitSwitchEnabledtrue);?P?
     m_leftElevatorMotor.configure(leftElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      rightElevatorMotorConfig
              .idleMode(IdleMode.kCoast)
        .follow(Constants.elevatorConstants.leftElevatorCAN, true);

       m_rightElevatorMotor.configure(rightElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 


     }
   //  public BooleanSupplier limitSwitchPressedSup = () -> (limitSwitch.isPressed());
     //positino is around 18.4,  do 10
     public void movePOS (){
    
     //  elevatorPID.setReference(-32,SparkBase.ControlType.kPosition);

            elevatorPID.setReference(level,SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0,0, ArbFFUnits.kVoltage);
    }
    public void setL1 (){
      level = elevatorConstants.level1Rotations; 
      SmartDashboard.putString("Elevator Set Level", "level One");
    }
    public void setL2 (){
      level = elevatorConstants.level2Rotations; 
      SmartDashboard.putString("Elevator Set Level", "level Two");
    }
    public void setL3 (){
      level = elevatorConstants.level3Rotations; 
      SmartDashboard.putString("Elevator Set Level", "level Three");
    }
    public void setL4 (){
      level = elevatorConstants.level4Rotations; 
      SmartDashboard.putString("Elevator Set Level", "level Four");
    }
    public void setIntake () {
      level = elevatorConstants.levelIntakeRotations; 
      SmartDashboard.putString("Elevator Set Level", "level intake");
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
// m_rightElevatorMotor.set(-speed);
//   m_leftElevatorMotor.set(speed);
   } else {
   //   m_rightElevatorMotor.setVoltage(3);
    //  m_leftElevatorMotor.setVoltage(3);
   }
}
}
}