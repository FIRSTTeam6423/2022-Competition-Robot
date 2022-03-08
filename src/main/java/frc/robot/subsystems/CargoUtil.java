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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CargoUtil extends SubsystemBase {
    //Shooter controllers
    private WPI_TalonSRX ballMagnet, lowIndexer, highIndexer;
    private CargoState state = CargoState.IDLE;
    private CANSparkMax shooter;
    private RelativeEncoder shooterEncoder;
    // private boolean shoot;
    private SparkMaxPIDController shooterPIDController; 
    //private PIDController pidController; 
    //Color detector
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

    
    Color detectedColor = m_colorSensor.getColor();

    //Limit switch
    private DigitalInput upperLimitSwitch;
    private DigitalInput lowerLimitSwitch;

    public CargoUtil() {
        // super(new PIDController(Constants.SHOOTER_P, Constants.SHOOTER_I,  Constants.SHOOTER_D));
        // shoot = false;

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

        upperLimitSwitch = new DigitalInput(Constants.UPPER_LIMIT_SWTICH);
        lowerLimitSwitch = new DigitalInput(Constants.LOWER_LIMIT_SWTICH);
    }
    
    public void operateBallMagnet(){
        ballMagnet.set(ControlMode.PercentOutput, Constants.BALL_MAGNET_OUTPUT);
    }

    public void reverseBallMagnet(){
        ballMagnet.set(ControlMode.PercentOutput, -Constants.BALL_MAGNET_OUTPUT);
    }

    public void stopBallMagnet(){
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
        // if (!shoot){
        //     enable();
        //     shoot = true;
        // }
        // setSetpoint(Constants.SHOOTER_RPM);
        // shooterPIDController.setReference(Constants.SHOOTER_RPM, CANSparkMax.ControlType.kVelocity);
        shooter.set(Constants.SHOOTER_VALUE);
    }

    public double getShooterRPM(){
        return shooterEncoder.getVelocity();
    }

    public void stopShooter(){
        // if (shoot){
        //     disable();
        //     shoot = false;
        // }
        // shooterPIDController.setReference(0.0, CANSparkMax.ControlType.kVelocity);
        shooter.set(0);
    }

    public void setState(CargoState newState){
        state = newState;

    }

    public void showLowerBallColor(){
        detectedColor = m_colorSensor.getColor();
        String color = "";
        boolean teamColorVsColor = false;

        if (detectedColor.red > Constants.RED_BALL_RED_VALUE && detectedColor.blue < Constants.RED_BALL_BLUE_VALUE){
            SmartDashboard.putString("Color detected", "RED");
            color = "RED";
        } else if (detectedColor.blue > Constants.BLUE_BALL_BLUE_VALUE && detectedColor.red < Constants.BLUE_BALL_RED_VALUE){
            SmartDashboard.putString("Color detected", "BLUE");
            color = "BLUE";
        } else {
            SmartDashboard.putString("Color detected", "NO COLOR DETECTED");
        }
        teamColorVsColor = color.equals(RobotContainer.getTeamColor());
        SmartDashboard.putBoolean("isTeamColor", teamColorVsColor);
    }

    /*public String detectLowerBallColor(){
        String color = "";
        if (detectedColor.red > Constants.RED_BALL_BLUE_VALUE && detectedColor.blue < Constants.RED_BALL_RED_VALUE){
            color = "RED";
        } else if (detectedColor.blue > Constants.BLUE_BALL_BLUE_VALUE && detectedColor.blue < Constants.BLUE_BALL_RED_VALUE){
            color = "BLUE"; 
        }
        return color;
    }*/

    public boolean detectLowerBall(){
        /*if ((detectedColor.red > Constants.RED_BALL_BLUE_VALUE && detectedColor.blue < Constants.RED_BALL_RED_VALUE) || 
        (detectedColor.blue > Constants.BLUE_BALL_BLUE_VALUE && detectedColor.blue < Constants.BLUE_BALL_RED_VALUE)){
            SmartDashboard.putString("Lower", "LOWER BALL DETECTED");
        } else{
            SmartDashboard.putString("Lower", "NO LOWER BALL DETECTED");
        }*/
        return lowerLimitSwitch.get();
    }

    public void showUpperBall(){
        if (upperLimitSwitch.get()){
            SmartDashboard.putString("Upper", "UPPER BALL DETECTED");
        } else {
            SmartDashboard.putString("Upper", "NO UPPER BALL DETECTED");
        }
    }

    public boolean detectUpperBall(){
        return upperLimitSwitch.get();
    }

    public CargoState returnState(){
        return state;
    }
  
    public void OperateCargo(){
        double rpm = getShooterRPM();
        switch(state){
            case INTAKE:
                if (detectUpperBall()){
                    stopLowIndexer();
                } else {
                    operateLowIndexer();
                }
                if (detectLowerBall()){
                    stopBallMagnet();
                } else {
                    operateBallMagnet();
                }
                // operateBallMagnet();
                // stopLowIndexer();
                stopHighIndexer();
                stopShooter();
                break;
            case INDEX:
                if (detectUpperBall()){
                    setState(CargoState.IDLE);
                }
                operateLowIndexer();
                stopHighIndexer();
                stopBallMagnet();
                stopShooter();
                break;
            case SPINUP:
                stopLowIndexer();
                stopHighIndexer();
                stopBallMagnet();
                operateShooter();
                if(rpm >= Constants.SHOOTER_RPM - Constants.SHOOTER_RPM_DEADBAND && 
                rpm <= Constants.SHOOTER_RPM + Constants.SHOOTER_RPM_DEADBAND)
                {
                    setState(CargoState.SHOOT);
                    
                }
                break;
            case SHOOT:
                operateHighIndexer();
                operateShooter();
                if (!detectUpperBall()){
                    operateLowIndexer();
                    operateBallMagnet();
                }
                break;
            case IDLE:
                stopLowIndexer();
                stopHighIndexer();
                stopShooter();
                stopBallMagnet();
                break;
            case SPIT:
                stopLowIndexer();
                stopHighIndexer();
                stopShooter();
                reverseBallMagnet();
        }
    }

    // public void toggleShooter(){
    //     if (isEnabled()){
    //         disable();
    //     } else {
    //         enable();
    //         setSetpoint(Constants.SHOOTER_RPM);
    //     }
    // }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        /** This is normally where we send important values to the SmartDashboard */
        SmartDashboard.putString("Shooter Mode  ::  ", state.toString());
        // SmartDashboard.putNumber("RPM", getShooterRPM());
        // SmartDashboard.putBoolean("PID Enabled", isEnabled());
        // SmartDashboard.putBoolean("Shooter Enabled", shoot);
        // SmartDashboard.putNumber("Setpoint", getSetpoint());

        showLowerBallColor();
        showUpperBall();
    }

    // @Override
    // protected void useOutput(double output, double setpoint) {
    //     shooter.set(output);
    //     System.out.println(setpoint + "OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO");
    // }

    // @Override
    // protected double getMeasurement() {
    //     return getShooterRPM();
    // }
}

