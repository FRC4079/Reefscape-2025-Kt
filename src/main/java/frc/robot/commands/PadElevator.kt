package frc.robot.commands

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Elevator
import frc.robot.utils.RobotParameters.ElevatorParameters.elevatorToBeSetState
import frc.robot.utils.RobotParameters.SwerveParameters.Thresholds.X_DEADZONE
import frc.robot.utils.RobotParameters.SwerveParameters.Thresholds.Y_DEADZONE
import frc.robot.utils.emu.ElevatorState
import kotlin.math.abs

/** Command to control the robot's swerve drive using a Logitech gaming pad.  */
class PadElevator(
    private val aacrn: XboxController?,
    private val calamityCow: XboxController,
) : Command() {
    /** Constructs a new PadDrive command.  */
    init {
        addRequirements(Elevator)
    }

    /**
     * Called every time the scheduler runs while the command is scheduled. This method retrieves the
     * current position from the gaming pad, calculates the rotation, logs the joystick values, and
     * sets the drive speeds for the swerve subsystem.
     */
    override fun execute() {
        // Code to manually move elevator
        //    Pair<Double, Double> position = positionSet(pad);
        //    Elevator.getInstance().moveElevator(position.getSecond());

        //    if (checkDPad(0)) {
        //      setElevatorState(L4).schedule();
        //    } else if (checkDPad(2)) {
        //      setElevatorState(ALGAE).schedule();
        //    } else if (checkDPad(4)) {
        //      setElevatorState(L2).schedule();
        //    } else if (checkDPad(6)) {
        //      setElevatorState(L1).schedule();
        //    }

        //    if (aacrn.getYButton()) {
        //      RobotParameters.CoralManipulatorParameters.coralState = CoralState.CORAL_INTAKE;
        //    } else {
        //      RobotParameters.CoralManipulatorParameters.coralState = CoralState.CORAL_HOLD;
        //    }

        // THIS IS WHEN WE HAVE TWO CONTROLLERS,
        // JAYDEN WILL CLICK A DPAD AND AUTOSCORE TAKES THIS VARIABLE AND GOES TO THAT HEIGHT

        when {
            checkDPad(0) -> {
                elevatorToBeSetState = ElevatorState.L4
            }
            checkDPad(2) -> {
                elevatorToBeSetState = ElevatorState.L3
            }
            checkDPad(4) -> {
                elevatorToBeSetState = ElevatorState.L2
            }
            checkDPad(6) -> {
                elevatorToBeSetState = ElevatorState.L1
            }
        }
    }

    /**
     * Check the state of the D-pad. The `index` is a value [0, 7] that corresponds to the
     * combinations on the D-pad. 0 represents just 'UP' being pressed, 1 is 'UP-RIGHT', 2 is just
     * 'RIGHT', 3 is 'RIGHT-DOWN', and so on.
     *
     *
     * This method can be used to see if a specific button on the D-pad is pressed.
     *
     * @param index The value to correspond to a D-pad combination.
     * @return If the specified combination is pressed.
     */
    fun checkDPad(index: Int): Boolean =
        if (index in 0..7) {
            (index * 45) == calamityCow.getPOV(0)
        } else {
            false
        }

    /**
     * Returns true when the command should end.
     *
     * @return Always returns false, as this command never ends on its own.
     */
    override fun isFinished(): Boolean = false

    companion object {
        /**
         * Sets the position based on the input from the Logitech gaming pad.
         *
         * @param pad The Logitech gaming pad.
         * @return The coordinate representing the position. The first element is the x-coordinate, and
         * the second element is the y-coordinate.
         */
        fun positionSet(pad: XboxController): Pair<Double, Double> {
            var x = pad.leftX
            if (abs(x) < X_DEADZONE) x = 0.0

            var y = pad.leftY
            if (abs(y) < Y_DEADZONE) y = 0.0

            return x to y
        }
    }
}
