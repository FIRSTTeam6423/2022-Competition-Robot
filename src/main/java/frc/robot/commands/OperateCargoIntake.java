package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.CargoUtil;
import frc.robot.RobotContainer;
import frc.robot.enums.CargoState;

public class OperateCargoIntake extends CommandBase{
    CargoUtil cu;

    public OperateCargoIntake(CargoUtil cu){
        this.cu = cu;
        addRequirements(this.cu);
    }

    @Override
    public void initialize() {
        //cu.setState(CargoState.IDLE);
        //SmartDashboard.putString("Shooter State", "Intake");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        /** 
        SmartDashboard.putString("Shooter State", "Index");
        if (RobotContainer.getOperatorAButton()){
            cu.setState(CargoState.INDEX);
            SmartDashboard.putString("Shooter State", "Index");
        } 
        if(RobotContainer.getOperatorXButton() && cu.detectLowerBall()) {
            cu.setState(CargoState.SPIT);
            SmartDashboard.putString("Shooter State", "Spit");
        } else if (RobotContainer.getOperatorXButton() && !cu.detectLowerBall()){
            cu.setState(CargoState.INTAKE);
            SmartDashboard.putString("Shooter State", "Intake");
        }
        if (RobotContainer.getOperatorYButton()){
            cu.setState(CargoState.IDLE);
            SmartDashboard.putString("Shooter State", "Idle");
        }
        **/
        cu.OperateCargo();
    }

    @Override
    public void end(boolean interrupted) {
        cu.setState(CargoState.IDLE);
        //SmartDashboard.putString("Shooter State", "Idle");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
