package frc.robot.util;

public enum CargoState {
    IDLE,INTAKE,INDEX,SPINUP,SHOOT, SPIT;
    /**Idle - No motors are running
     * Intake - Ball magnet is running
     * Index - Low Indexer is running
     * Spinup - Shooter is speeding up
     * Shoot - Shooter is at spead and High Indexer is running
     * Spit - Ball magnet runs in reverse
     */
}
