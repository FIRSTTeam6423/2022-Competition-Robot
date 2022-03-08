package frc.robot.util;

public enum CargoState {
    IDLE,INTAKE,SPINUP,SHOOT, SPIT;
    /**Idle - No motors are running
     * Intake - Ball magnet is running
     * Spinup - Shooter is speeding up
     * Shoot - Shooter is at spead and High Indexer is running
     * Spit - Ball magnet runs in reverse
     */
}
