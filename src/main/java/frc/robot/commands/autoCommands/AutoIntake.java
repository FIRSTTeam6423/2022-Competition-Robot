package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.CargoUtil;
import frc.robot.util.CargoState;
import frc.robot.RobotContainer;

public class AutoIntake extends CommandBase{
    CargoUtil cu;
    private boolean done;

    public AutoIntake(CargoUtil cu){
        this.cu = cu;
        addRequirements(this.cu);
    }

    @Override
    public void initialize() {
        cu.setState(CargoState.INTAKE);

        done = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if ((cu.detectLowerBallColor().equals(RobotContainer.getTeamColor()))){
            cu.setState(CargoState.INDEX);
        } else if ((cu.detectLowerBallColor().equals(""))) {
            cu.setState(CargoState.INTAKE);
        } else {
            cu.setState(CargoState.SPIT);
        }
        if (cu.detectUpperBall()){
            cu.setState(CargoState.IDLE);
            done = true;
        }
        cu.OperateCargo();
    }

    @Override
    public void end(boolean interrupted) {
        cu.setState(CargoState.IDLE);
        cu.OperateCargo();
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
