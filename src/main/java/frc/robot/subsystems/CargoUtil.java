package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CargoUtil extends SubsystemBase{
    //Shooter controllers
    private WPI_TalonSRX ballMagnet, indexer;
    private CANSparkMax shooter;

    public CargoUtil() {
        ballMagnet = new WPI_TalonSRX(Constants.BALL_MAGNET);
        indexer = new WPI_TalonSRX(Constants.INDEXER);
        shooter = new CANSparkMax(Constants.SHOOTER, MotorType.kBrushless);
    }
    
    public void OperateBallMagnet(){
        ballMagnet.set(ControlMode.PercentOutput, Constants.BALL_MAGNET_OUTPUT);
    }

    public void StopBallMagent(){
        ballMagnet.set(ControlMode.PercentOutput, 0);
    }

    public void OperateIndexer(){
        indexer.set(ControlMode.PercentOutput, Constants.INDEXER_OUTPUT);
    }

    public void StopIndexer(){
        indexer.set(ControlMode.PercentOutput, 0);
    }

    public void OperateShooter(){

    }
}
