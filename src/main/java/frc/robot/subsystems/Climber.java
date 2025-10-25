package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
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
        public RelativeEncoder climbEncoder = m_climberMotor.getEncoder(); 

    public Climber(){
            climberMotorConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast);
            m_climberMotor.configure(climberMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void climb(){
        m_climberMotor.set(-0.3);
    }
    public void deClimb(){
        m_climberMotor.set(0.3);
    }
    public void stop(){
        m_climberMotor.stopMotor();
    }
    public double getClimb(){
        return climbEncoder.getPosition();
    
    }
}
