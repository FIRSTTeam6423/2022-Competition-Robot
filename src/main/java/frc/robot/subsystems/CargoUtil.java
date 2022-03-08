package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.util.CargoState;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CargoUtil extends SubsystemBase {
    //Shooter controllers
    private WPI_TalonSRX ballMagnet, lowIndexer, highIndexer;
    private CargoState state = CargoState.IDLE;
    private CANSparkMax shooter;
    private RelativeEncoder shooterEncoder;
    // PID not currently used, but may be used in future so do not delete
    //private SparkMaxPIDController shooterPIDController; 

    //Limit switch
    private DigitalInput upperLimitSwitch;
    private DigitalInput lowerLimitSwitch;

    public CargoUtil() {

        ballMagnet = new WPI_TalonSRX(Constants.BALL_MAGNET);
        lowIndexer = new WPI_TalonSRX(Constants.LOW_INDEXER);
        highIndexer = new WPI_TalonSRX(Constants.HIGH_INDEXER);
        shooter = new CANSparkMax(Constants.SHOOTER, MotorType.kBrushless);

        shooter.setInverted(true);

        shooterEncoder = shooter.getEncoder();
        //shooterPIDController = shooter.getPIDController();

        //shooterPIDController.setP(Constants.SHOOTER_P);
        //shooterPIDController.setI(Constants.SHOOTER_I);
        //shooterPIDController.setD(Constants.SHOOTER_D);
        //shooterPIDController.setFF(Constants.SHOOTER_F);

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
        // shooterPIDController.setReference(Constants.SHOOTER_RPM, CANSparkMax.ControlType.kVelocity);
        shooter.set(Constants.SHOOTER_VALUE);
    }

    public double getShooterRPM(){
        return shooterEncoder.getVelocity();
    }

    public void stopShooter(){
        // shooterPIDController.setReference(0.0, CANSparkMax.ControlType.kVelocity);
        shooter.set(0);
    }

    public void setState(CargoState newState){
        state = newState;

    }

    public boolean detectLowerBall(){
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
                stopHighIndexer();
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

    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        /** This is normally where we send important values to the SmartDashboard */
        SmartDashboard.putString("Shooter Mode  ::  ", state.toString());

        showUpperBall();
    }
}

