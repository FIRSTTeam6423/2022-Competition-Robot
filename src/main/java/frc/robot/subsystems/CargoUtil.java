package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CargoUtil extends SubsystemBase{
    //Shooter controllers
    private WPI_TalonSRX ballMagnet, indexer;
    private CANSparkMax shooter;

    //Color detector
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

    
    Color detectedColor = m_colorSensor.getColor();
    int proximity = m_colorSensor.getProximity();

    public CargoUtil() {
        //ballMagnet = new WPI_TalonSRX(Constants.BALL_MAGNET);
        //indexer = new WPI_TalonSRX(Constants.INDEXER);
        //shooter = new CANSparkMax(Constants.SHOOTER, MotorType.kBrushless);
    }
    
    public void OperateBallMagnet(){
        //ballMagnet.set(ControlMode.PercentOutput, Constants.BALL_MAGNET_OUTPUT);
    }

    public void StopBallMagent(){
        //ballMagnet.set(ControlMode.PercentOutput, 0);
    }

    public void OperateIndexer(){
        //indexer.set(ControlMode.PercentOutput, Constants.INDEXER_OUTPUT);
    }

    public void StopIndexer(){
        //indexer.set(ControlMode.PercentOutput, 0);
    }

    public void OperateShooter(){

    }

    /**
     * Constantly check the rgb values read by the color sensor
     * If they match the values of red cargo, display "RED" on the dashboard
     * If they match the values of blue cargo, display "BLUE" on the dashboard
     * If the rgb values match neither types of cargo, display "NO COLOR DETECTED" on the dashboard
     */
    public void detectBallColor(){
        //if (detectedColor.red > 0.55 && detectedColor.blue < 0.1){
        //    SmartDashboard.putString("color detected", "RED");
        //} else if (detectedColor.blue > 0.3){
        //    SmartDashboard.putString("color detected", "BLUE");
        //} else {
        //    SmartDashboard.putString("color detected", "NO COLOR DETECTED");
        //}
    }
}
