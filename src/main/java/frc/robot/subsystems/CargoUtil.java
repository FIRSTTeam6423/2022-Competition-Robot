package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.enums.CargoState;

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
    int proximity = m_colorSensor.getProximity();

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
    
    public void OperateBallMagnet(){
        ballMagnet.set(ControlMode.PercentOutput, Constants.BALL_MAGNET_OUTPUT);
    }

    public void StopBallMagent(){
        ballMagnet.set(ControlMode.PercentOutput, 0);
    }

    public void OperateLowIndexer(){
        lowIndexer.set(ControlMode.PercentOutput, Constants.INDEXER_OUTPUT);
    }

    public void OperateHighIndexer(){
        highIndexer.set(ControlMode.PercentOutput, Constants.INDEXER_OUTPUT);
    }

    public void StopLowIndexer(){
        lowIndexer.set(ControlMode.PercentOutput, 0);

    }
    public void StopHighIndexer(){
        highIndexer.set(ControlMode.PercentOutput, 0);
    }


    public void OperateShooter(){
        shooterPIDController.setReference(Constants.SHOOTER_RPM, CANSparkMax.ControlType.kVelocity);
    }

    public void StopShooter(){
        shooterPIDController.setReference(0.0, CANSparkMax.ControlType.kVelocity);
    }

    public void SetState(CargoState newState){
        state = newState;

    }
  
    public void OperateCargo(){
        switch(state){
            case INTAKE:
                //Red ball detected
                if (detectedColor.red > 0.55 && detectedColor.blue < 0.1){
                    StopLowIndexer();
                    StopHighIndexer();
                    StopBallMagent();
                    StopShooter();
                //Blue ball detected
                } else if (detectedColor.blue > 0.3){    
                    StopLowIndexer();
                    StopHighIndexer();
                    StopBallMagent();
                    StopShooter();
                //No ball detected
                } else {
                    OperateBallMagnet();
                    OperateLowIndexer();
                    StopHighIndexer();
                    StopShooter();
                }
                break;
            case INDEX:
                //Ball detected
                if (limitSwitch.get()){
                    StopLowIndexer();
                    StopHighIndexer();
                    StopBallMagent();
                    StopShooter();
                //No ball detected
                } else {
                    OperateLowIndexer();
                    StopHighIndexer();
                    StopBallMagent();
                    StopShooter();
                }
                break;
            case SPINUP:
                StopLowIndexer();
                StopHighIndexer();
                StopBallMagent();
                OperateShooter();
                break;
            case SHOOT:
                StopLowIndexer();
                StopBallMagent();
                OperateHighIndexer();
                OperateShooter();
                break;
            case IDLE:
                StopLowIndexer();
                StopHighIndexer();
                StopShooter();
                StopBallMagent();
                break;
        }
    }

  /** 
     * Constantly check the rgb values read by the color sensor
     * If they match the values of red cargo, display "RED" on the dashboard
     * If they match the values of blue cargo, display "BLUE" on the dashboard
     * If the rgb values match neither types of cargo, display "NO COLOR DETECTED" on the dashboard
    */
    public void detectBallColor(){
        if (detectedColor.red > 0.55 && detectedColor.blue < 0.1){
            SmartDashboard.putString("color detected", "RED");
        } else if (detectedColor.blue > 0.3){
            SmartDashboard.putString("color detected", "BLUE");
        } else {
            SmartDashboard.putString("color detected", "NO COLOR DETECTED");
        }

    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        /** This is normally where we send important values to the SmartDashboard */
        SmartDashboard.putString("Shooter Mode  ::  ", state.toString());
    }
}

