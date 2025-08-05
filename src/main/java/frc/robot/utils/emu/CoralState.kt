package frc.robot.utils.emu

import frc.robot.subsystems.Coral

/**
 * Enum class representing the states of the coral manipulator.
 *
 * @property block The function associated with the coral state
 */
enum class CoralState(
    @JvmField val block: Runnable,
) {
    /** Represents the state when the coral manipulator is intaking a coral piece. */
    CORAL_INTAKE(Runnable { Coral.startCoralIntake() }),

    /** Represents the state when the coral manipulator is holding a coral piece. */
    CORAL_HOLD(Runnable { Coral.stopMotors() }),

    /** Represents the state when the coral manipulator is slowing down to hold a coral piece. */
    CORAL_SLOW(Runnable { Coral.slowCoralIntake() }),

    /** Represents the state when the coral manipulator is releasing a coral piece. */
    CORAL_RELEASE(Runnable { Coral.scoreCoral() }),

    /** Represents the state when the coral manipulator is intaking algae. */
    ALGAE_INTAKE(Runnable { Coral.algaeIntake() }),

    /** Represents the state when the coral manipulator is holding algae. */
    ALGAE_HOLD(Runnable { Coral.slowAlgaeScoreMotors() }),

    /** Represents the state when the coral manipulator is releasing algae. */
    ALGAE_RELEASE(Runnable { Coral.ejectAlgae() }),
}
