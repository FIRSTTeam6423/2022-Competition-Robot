package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;
import frc.robot.util.CargoState;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CargoUtil extends SubsystemBase {
    //Shooter controllers
    private WPI_TalonSRX ballMagnet, lowIndexer, highIndexer;
    private CargoState state = CargoState.IDLE;
    private Timer delay_between_shots_timer;
    private Timer first_delay_timer;


    //Limit switch
    private DigitalInput upperLimitSwitch;
    private DigitalInput lowerLimitSwitch;

    public CargoUtil() {

        ballMagnet = new WPI_TalonSRX(Constants.BALL_MAGNET);
        lowIndexer = new WPI_TalonSRX(Constants.LOW_INDEXER);
        highIndexer = new WPI_TalonSRX(Constants.HIGH_INDEXER);

        upperLimitSwitch = new DigitalInput(Constants.UPPER_LIMIT_SWTICH);
        lowerLimitSwitch = new DigitalInput(Constants.LOWER_LIMIT_SWTICH);

        delay_between_shots_timer = new Timer();
        delay_between_shots_timer.reset();

        ballMagnet.setInverted(true);
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
                delay_between_shots_timer.reset();
                stopLowIndexer();
                stopHighIndexer();
                stopBallMagnet();
                if(RobotContainer.getIsReadyToShoot()){
                    delay_between_shots_timer.start();
                    setState(CargoState.SHOOT);
                }
                break;
            case SHOOT:
                if (delay_between_shots_timer.get() > Constants.SHOOT_TIME_INTERVAL){
                    operateHighIndexer();
                }
                if (delay_between_shots_timer.get() > (Constants.SHOOT_TIME_INTERVAL + Constants.SHOOT_DELAY ) && !detectUpperBall()){
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
        // SmartDashboard.putString("Shooter Mode  ::  ", state.toString());

        SmartDashboard.putBoolean("1 Ball Loaded", detectLowerBall() ^ detectUpperBall());
        SmartDashboard.putBoolean("2 Balls Loaded", detectUpperBall() && detectLowerBall());
    }
}
