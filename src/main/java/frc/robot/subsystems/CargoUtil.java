package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.enums.CargoState;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CargoUtil extends SubsystemBase{
    //Shooter controllers
    private WPI_TalonSRX ballMagnet, lowIndexer, highIndexer;
    private CargoState state = CargoState.IDLE;
    private CANSparkMax shooter;
    private SparkMaxPIDController shooterPIDController; 

    //Color detector
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

    
    Color detectedColor = m_colorSensor.getColor();
    //int proximity = m_colorSensor.getProximity();

    //Limit switch
    private DigitalInput limitSwitch;

    public CargoUtil() {
        ballMagnet = new WPI_TalonSRX(Constants.BALL_MAGNET);
        lowIndexer = new WPI_TalonSRX(Constants.LOW_INDEXER);
        highIndexer = new WPI_TalonSRX(Constants.HIGH_INDEXER);
        shooter = new CANSparkMax(Constants.SHOOTER, MotorType.kBrushless);
        shooterPIDController = shooter.getPIDController();

        shooterPIDController.setP(Constants.SHOOTER_P);
        shooterPIDController.setI(Constants.SHOOTER_I);
        shooterPIDController.setD(Constants.SHOOTER_D);
        shooterPIDController.setFF(Constants.SHOOTER_F);

        limitSwitch = new DigitalInput(Constants.LIMIT_SWTICH);
    }
    
    public void operateBallMagnet(){
        ballMagnet.set(ControlMode.PercentOutput, Constants.BALL_MAGNET_OUTPUT);
    }

    public void stopBallMagent(){
        ballMagnet.set(ControlMode.PercentOutput, 0);
    }

    public void operateLowIndexer(){
        lowIndexer.set(ControlMode.PercentOutput, Constants.INDEXER_OUTPUT);
    }

    public void operateHighIndexer(){
        highIndexer.set(ControlMode.PercentOutput, Constants.INDEXER_OUTPUT);
    }

    public void stopLowIndexer(){
        lowIndexer.set(ControlMode.PercentOutput, 0);

    }
    public void stopHighIndexer(){
        highIndexer.set(ControlMode.PercentOutput, 0);
    }


    public void operateShooter(){
        shooterPIDController.setReference(Constants.SHOOTER_RPM, CANSparkMax.ControlType.kVelocity);
    }

    public void stopShooter(){
        shooterPIDController.setReference(0.0, CANSparkMax.ControlType.kVelocity);
    }

    public void setState(CargoState newState){
        state = newState;

    }

    public void detectBallColor(){
        detectedColor = m_colorSensor.getColor();

        if (detectedColor.red > 0.55 && detectedColor.blue < 0.1){
            SmartDashboard.putString("color detected", "RED");
        } else if (detectedColor.blue > 0.3){
            SmartDashboard.putString("color detected", "BLUE");
        } else {
            SmartDashboard.putString("color detected", "NO COLOR DETECTED");
        }
        SmartDashboard.putNumber("red", detectedColor.red);
    }

    public void detectBall(){
        if (limitSwitch.get()){
            SmartDashboard.putString("ball detected", "BALL DETECTED");
        } else {
            SmartDashboard.putString("ball detected", "NO BALLs DETECTED");
        }
    }
  
    public void OperateCargo(){
        switch(state){
            case INTAKE:
                //Red ball detected
                if (detectedColor.red > 0.55 && detectedColor.blue < 0.1){
                    stopLowIndexer();
                    stopHighIndexer();
                    stopBallMagent();
                    stopShooter();
                //Blue ball detected
                } else if (detectedColor.blue > 0.3){    
                    stopLowIndexer();
                    stopHighIndexer();
                    stopBallMagent();
                    stopShooter();
                //No ball detected
                } else {
                    operateBallMagnet();
                    operateLowIndexer();
                    stopHighIndexer();
                    stopShooter();
                }
                break;
            case INDEX:
                //Ball detected
                if (limitSwitch.get()){
                    stopLowIndexer();
                    stopHighIndexer();
                    stopBallMagent();
                    stopShooter();
                //No ball detected
                } else {
                    operateLowIndexer();
                    stopHighIndexer();
                    stopBallMagent();
                    stopShooter();
                }
                break;
            case SPINUP:
                stopLowIndexer();
                stopHighIndexer();
                stopBallMagent();
                operateShooter();
                break;
            case SHOOT:
                stopLowIndexer();
                stopBallMagent();
                operateHighIndexer();
                operateShooter();
                break;
            case IDLE:
                stopLowIndexer();
                stopHighIndexer();
                stopShooter();
                stopBallMagent();
                break;
        }
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        /** This is normally where we send important values to the SmartDashboard */
        SmartDashboard.putString("Shooter Mode  ::  ", state.toString());
    }
}

