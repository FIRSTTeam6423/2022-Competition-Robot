package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.CargoUtil;
import frc.robot.util.CargoState;

public class AutoIntake extends CommandBase{
    CargoUtil cu;

    public AutoIntake(CargoUtil cu){
        this.cu = cu;
        addRequirements(this.cu);
    }

    @Override
    public void initialize() {
        cu.setState(CargoState.INTAKE);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        cu.setState(CargoState.INTAKE);
        cu.OperateCargo();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        cu.setState(CargoState.SPINUP);
        cu.OperateCargo();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
