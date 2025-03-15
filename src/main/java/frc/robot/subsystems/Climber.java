package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    public static SparkMax m_climberMotor = new SparkMax(Constants.climberConstants.climberMotorCAN, MotorType.kBrushless);
    SparkMaxConfig climberMotorConfig = new SparkMaxConfig();
    static SparkClosedLoopController climberPID = m_climberMotor.getClosedLoopController();
    
    public Climber(){
            climberMotorConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast);
            climberMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.0001,0,0);

            m_climberMotor.configure(climberMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void climb(){
        climberPID.setReference(10, SparkBase.ControlType.kPosition);
    }
    public void deClimb(){
        climberPID.setReference(0, SparkBase.ControlType.kPosition);
    }
}
