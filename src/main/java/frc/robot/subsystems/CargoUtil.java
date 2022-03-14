package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;
import frc.robot.util.CargoState;


import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CargoUtil extends SubsystemBase {
    //Shooter controllers
    private WPI_TalonSRX ballMagnet, lowIndexer, highIndexer;
    private CargoState state = CargoState.IDLE;

    //Limit switch
    private DigitalInput upperLimitSwitch;
    private DigitalInput lowerLimitSwitch;

    public CargoUtil() {

        ballMagnet = new WPI_TalonSRX(Constants.BALL_MAGNET);
        lowIndexer = new WPI_TalonSRX(Constants.LOW_INDEXER);
        highIndexer = new WPI_TalonSRX(Constants.HIGH_INDEXER);

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

    public void reverseLowIndexer(){
        lowIndexer.set(ControlMode.PercentOutput, -Constants.INDEXER_OUTPUT);
    }

    public void operateHighIndexer(){
        highIndexer.set(ControlMode.PercentOutput, Constants.INDEXER_OUTPUT);
    }

    public void reverseHighIndexer(){
        highIndexer.set(ControlMode.PercentOutput, -Constants.INDEXER_OUTPUT);
    }

    public void stopLowIndexer(){
        lowIndexer.set(ControlMode.PercentOutput, 0);

    }

    public void stopHighIndexer(){
        highIndexer.set(ControlMode.PercentOutput, 0);
    }


    public void setState(CargoState newState){
        state = newState;
    }

    public boolean detectLowerBall(){
        return !lowerLimitSwitch.get();
    }

    public boolean detectUpperBall(){
        return upperLimitSwitch.get();
    }

    public CargoState returnState(){
        return state;
    }
  
    public void OperateCargo(){
        switch(state){
            case INTAKE:
                if (detectUpperBall()){
                    stopLowIndexer();
                    if (detectLowerBall()){
                        stopBallMagnet();
                    } else {
                        operateBallMagnet();
                    }
                } else {
                    operateLowIndexer();
                    operateBallMagnet();
                }
                stopHighIndexer();
                break;
            case SPINUP:
                stopBallMagnet();
                stopHighIndexer();
                stopLowIndexer();
            case SHOOT:
                operateHighIndexer();
                if (!detectUpperBall()){
                    operateLowIndexer();
                    operateBallMagnet();
                }
                break;
            case IDLE:
                stopLowIndexer();
                stopHighIndexer();
                stopBallMagnet();
                break;
            case SPIT:
                reverseLowIndexer();
                reverseHighIndexer();
                reverseBallMagnet();
        }
    }

    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        /** This is normally where we send important values to the SmartDashboard */
        SmartDashboard.putString("Shooter Mode  ::  ", state.toString());

        SmartDashboard.putBoolean("High Ball", detectUpperBall());
        SmartDashboard.putBoolean("Low Ball", detectLowerBall());
    }
}

