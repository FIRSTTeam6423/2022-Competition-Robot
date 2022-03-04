package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.CargoState;

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
    
    private double shooterP;
    private double shooterI;
    private double shooterD;
    private double shooterF;

    //Color detector
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

    
    Color detectedColor = m_colorSensor.getColor();
    Integer detectedLength = m_colorSensor.getProximity();

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

        shooterPIDController.setP(shooterP);
        shooterPIDController.setI(shooterI);
        shooterPIDController.setD(shooterD);
        shooterPIDController.setFF(shooterF);

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
        shooterPIDController.setReference(Constants.SHOOTER_RPM, CANSparkMax.ControlType.kVelocity);
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

    public void detectLowerBall(){
        if (detectedLength > Constants.BALL_DISTANCE ){
            SmartDashboard.putString("Lower", "LOWER BALL DETECTED");
        } else {
            SmartDashboard.putString("Lower", "NO LOWER BALL DETECTED");
        }
    }

    public void showUpperBall(){
        if (limitSwitch.get()){
            SmartDashboard.putString("Upper", "UPPER BALL DETECTED");
        } else {
            SmartDashboard.putString("Upper", "NO UPPER BALL DETECTED");
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
                //TODO: Fix color sensor   
                //if (detectLowerBall()){
                    //setState(CargoState.IDLE);
                //}
                operateBallMagnet();
                stopLowIndexer();
                stopHighIndexer();
                stopShooter();
                break;
            case INDEX:
                if (detectUpperBall()){
                    setState(CargoState.IDLE);
                }
                operateLowIndexer();
                stopHighIndexer();
                stopBallMagent();
                stopShooter();
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
                break;
            case SHOOT:
                operateLowIndexer();
                operateBallMagnet();
                operateHighIndexer();
                operateShooter();
                break;
            case IDLE:
                stopLowIndexer();
                stopHighIndexer();
                stopShooter();
                stopBallMagent();
                break;
            case SPIT:
                stopLowIndexer();
                stopHighIndexer();
                stopShooter();
                reverseBallMagnet();
        }
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        /** This is normally where we send important values to the SmartDashboard */
        Integer detectedLength = m_colorSensor.getProximity();
        SmartDashboard.putString("Shooter Mode  ::  ", state.toString());
        SmartDashboard.putNumber("RPM", getShooterRPM());
        shooterP = SmartDashboard.getNumber("P", shooterP);
        shooterI = SmartDashboard.getNumber("I", shooterI);
        shooterD = SmartDashboard.getNumber("D", shooterD);
        shooterF = SmartDashboard.getNumber("F", shooterF);
        SmartDashboard.putNumber("P", shooterP);
        SmartDashboard.putNumber("I", shooterI);
        SmartDashboard.putNumber("D", shooterD);
        SmartDashboard.putNumber("F", shooterF);

        shooterPIDController.setP(shooterP);
        shooterPIDController.setI(shooterI);
        shooterPIDController.setD(shooterD);
        shooterPIDController.setFF(shooterF);

        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Distance", detectedLength);

        showLowerBallColor();
        showUpperBall();
        SmartDashboard.putBoolean("Digital Input", limitSwitch.get());
    }
}

