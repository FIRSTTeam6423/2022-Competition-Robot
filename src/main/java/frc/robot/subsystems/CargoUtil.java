package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.enums.CargoState;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
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
    private RelativeEncoder shooterEncoder;
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

        shooter.setInverted(true);

        shooterEncoder = shooter.getEncoder();
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

    public void reverseBallMagnet(){
        ballMagnet.set(ControlMode.PercentOutput, -Constants.BALL_MAGNET_OUTPUT);
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
        //shooterPIDController.setReference(Constants.SHOOTER_RPM, CANSparkMax.ControlType.kVelocity);
        shooter.set(0.5);
    }
    
    public double getShooterRPM(){
        return shooterEncoder.getVelocity();
    }

    public void stopShooter(){
        shooterPIDController.setReference(0.0, CANSparkMax.ControlType.kVelocity);
    }

    public void setState(CargoState newState){
        state = newState;

    }

    public void showLowerBallColor(){
        detectedColor = m_colorSensor.getColor();
        String color = "";
        boolean teamColorVsColor = false;

        if (detectedColor.red > Constants.RED_BALL_RED_VALUE && detectedColor.blue < Constants.RED_BALL_BLUE_VALUE){
            SmartDashboard.putString("color detected", "RED");
            color = "RED";
        } else if (detectedColor.blue > Constants.BLUE_BALL_BLUE_VALUE && detectedColor.red < Constants.BLUE_BALL_RED_VALUE){
            SmartDashboard.putString("color detected", "BLUE");
            color = "BLUE";
        } else {
            SmartDashboard.putString("color detected", "NO COLOR DETECTED");
        }
        teamColorVsColor = color.equals(RobotContainer.getTeamColor());
        SmartDashboard.putBoolean("isTeamColor", teamColorVsColor);
    }

    public String detectLowerBallColor(){
        String color = "";
        if (detectedColor.red > Constants.RED_BALL_BLUE_VALUE && detectedColor.blue < Constants.RED_BALL_RED_VALUE){
            color = "RED";
        } else if (detectedColor.blue > Constants.BLUE_BALL_BLUE_VALUE && detectedColor.blue < Constants.BLUE_BALL_RED_VALUE){
            color = "BLUE"; 
        }
        return color;
    }

    public boolean detectLowerBall(){
        boolean ball = false;
        if (detectedColor.red > Constants.RED_BALL_BLUE_VALUE && detectedColor.blue < Constants.RED_BALL_RED_VALUE){
            ball = true;
        } else if (detectedColor.blue > Constants.BLUE_BALL_BLUE_VALUE && detectedColor.blue < Constants.BLUE_BALL_RED_VALUE){
            ball = true; 
        }
        return ball;
    }

    public void showUpperBall(){
        if (limitSwitch.get()){
            SmartDashboard.putString("ball detected", "BALL DETECTED");
        } else {
            SmartDashboard.putString("ball detected", "NO BALLs DETECTED");
        }
    }

    public boolean detectUpperBall(){
        return limitSwitch.get();
    }

    public CargoState returnState(){
        return state;
    }
  
    public void OperateCargo(){
        double rpm = getShooterRPM();
        switch(state){
            case INTAKE:
                if (detectLowerBall()){
                    setState(CargoState.IDLE);
                }
                operateBallMagnet();
                stopLowIndexer();
                stopHighIndexer();
                stopShooter();
                //SmartDashboard.putString("Shoot State", "Intake");
                break;
            case INDEX:
                if (detectUpperBall()){
                    setState(CargoState.IDLE);
                }
                operateLowIndexer();
                stopHighIndexer();
                stopBallMagent();
                stopShooter();
                //SmartDashboard.putString("Shoot State", "Index");
                break;
            case SPINUP:
                stopLowIndexer();
                stopHighIndexer();
                stopBallMagent();
                operateShooter();
                if(rpm >= Constants.SHOOTER_RPM - Constants.SHOOTER_RPM_DEADBAND && 
                rpm <= Constants.SHOOTER_RPM + Constants.SHOOTER_RPM_DEADBAND)
                {
                    setState(CargoState.SHOOT);
                }
                //SmartDashboard.putString("Shoot State", "Intake");
                break;
            case SHOOT:
                stopLowIndexer();
                stopBallMagent();
                operateHighIndexer();
                operateShooter();
                    //SmartDashboard.putString("Shoot State", "Shoot");
                break;
            case IDLE:
                stopLowIndexer();
                stopHighIndexer();
                stopShooter();
                stopBallMagent();
                //SmartDashboard.putString("Shoot State", "Idle");
                break;
            case SPIT:
                stopLowIndexer();
                stopHighIndexer();
                stopShooter();
                reverseBallMagnet();
                //SmartDashboard.putString("Shoot State", "Spit");
        }
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        /** This is normally where we send important values to the SmartDashboard */
        SmartDashboard.putString("Shooter Mode  ::  ", state.toString());
        SmartDashboard.putNumber("RPM", getShooterRPM());
        showLowerBallColor();
        showUpperBall();
    }
}

