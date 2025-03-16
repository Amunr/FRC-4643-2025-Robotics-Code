package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import java.util.function.BooleanSupplier;

import javax.lang.model.util.ElementScanner14;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.coralConstants;

public class Coral extends SubsystemBase {
    public static SparkMax m_leftCoralMotor = new SparkMax(Constants.coralConstants.leftCoralCAN, MotorType.kBrushless);
    public static SparkMax m_rightCoralMotor = new SparkMax(Constants.coralConstants.rightCoralCAN, MotorType.kBrushless);
    SparkMaxConfig leftCoralConfig = new SparkMaxConfig();
    SparkMaxConfig rightCoralConfig = new SparkMaxConfig();

    // Beam Breaks
    public static AnalogInput coralBeamBreak = new AnalogInput(coralConstants.frontBeamBreakPort); 
    public static AnalogInput intakeBeamBreak = new AnalogInput(coralConstants.backBeamBreakPort); 
    // public BooleanSupplier intakeBeamBreakStatus = () -> (intakeBeamBreak.getValue() < 10);
    public BooleanSupplier coralBeamBreakStatus = () -> (coralBeamBreak.getValue() < 10);
    public BooleanSupplier coralBeamBreakStatusINV = () -> (coralBeamBreak.getValue() > 10 );
    public Boolean peiceHeld;
    public Coral () {
         peiceHeld =  SmartDashboard.getBoolean("Preload", true);
        leftCoralConfig
        .idleMode(IdleMode.kBrake)
        .inverted(false);
        m_leftCoralMotor.configure(leftCoralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightCoralConfig
        .idleMode(IdleMode.kBrake)
        .inverted(true);
        m_rightCoralMotor.configure(rightCoralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      
    }

    public void startIntake(){
        m_leftCoralMotor.set(0.5);
        m_rightCoralMotor.set(0.5);

    }
    public void slowIntake(){
        m_leftCoralMotor.set(0.08);
        m_rightCoralMotor.set(0.08);
    }
    public void outtake(){
        m_leftCoralMotor.set(1);
        m_rightCoralMotor.set(1);
        peiceHeld = false;
    }

    public void peiceHeldTrue(){
        peiceHeld = true;
    }
    public void reverseIntake(){
        m_leftCoralMotor.set(-0.5);
        m_leftCoralMotor.set(-0.5);

    }
    public void stopCoralMotor(){
        m_leftCoralMotor.stopMotor();
        m_rightCoralMotor.stopMotor();

    }
    public boolean coralBeamBreakStatus(){
        if(coralBeamBreak.getValue() > 10){
            return false;
        } else {
            return true;
        }

    }

    public boolean intakeBeamBreakStatus(){
        if(intakeBeamBreak.getValue() > 10){
            return false;
        } else {
            return true;
        }
    }

    public int coroalBeamBreakStatusINT(){
        return coralBeamBreak.getValue();
    } 
}
