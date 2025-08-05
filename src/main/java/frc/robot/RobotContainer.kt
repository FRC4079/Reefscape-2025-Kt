package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.commands.Kommand
import frc.robot.commands.Kommand.drive
import frc.robot.commands.Kommand.hasPieceFalse
import frc.robot.commands.Kommand.moveElevatorState
import frc.robot.commands.Kommand.padElevator
import frc.robot.commands.Kommand.reverseIntake
import frc.robot.commands.Kommand.setElevatorState
import frc.robot.commands.sequencing.Sequences
import frc.robot.commands.sequencing.Sequences.fullScore
import frc.robot.commands.sequencing.Sequences.fullScoreAuto
import frc.robot.subsystems.Algae
import frc.robot.subsystems.Coral
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Swerve
import frc.robot.utils.emu.Button
import frc.robot.utils.emu.Direction
import frc.robot.utils.emu.ElevatorState
import frc.robot.utils.pingu.Bingu.bind
import frc.robot.utils.pingu.Bingu.bindings
import frc.robot.utils.pingu.CommandPingu
import java.util.function.BooleanSupplier

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
    val networkChooser: SendableChooser<Command?>
    val aacrn: XboxController
    val calamityCow: XboxController

    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        aacrn = XboxController(0)
        calamityCow = XboxController(1)

        Elevator.defaultCommand = padElevator(aacrn, calamityCow)
        Coral
        Algae
        Swerve.defaultCommand = drive(aacrn)

        CommandPingu()
            .bind("ScoreL4Left", fullScoreAuto(Direction.LEFT))
            .bind("ScoreL4Right", fullScoreAuto(Direction.RIGHT))
            .bind("HasPieceFalse", hasPieceFalse())
            .bind("MoveElevatorL4Auto", moveElevatorState(ElevatorState.L4))
            .bind("MoveElevatorDefaultAuto", moveElevatorState(ElevatorState.DEFAULT))
            .bind("SetL1", setElevatorState(ElevatorState.L1))
            .bind("SetL2", setElevatorState(ElevatorState.L2))
            .bind("SetL3", setElevatorState(ElevatorState.L3))
            .bind("SetL4", setElevatorState(ElevatorState.L4))
            .bind("MoveElevatorDown", setElevatorState(ElevatorState.DEFAULT))

        networkChooser = AutoBuilder.buildAutoChooser()

        configureBindings()

        //    networkChooser.addDefaultOption("Straight Auto", new PathPlannerAuto("Straight Auto"));
        //    networkChooser.addOption("Straight Auto", new InstantCommand());
    }

    /**
     * Use this method to define your trigger -> command mappings. Triggers can be created via the
     * [edu.wpi.first.wpilibj2.command.button.JoystickButton] constructor with an arbitrary predicate, or via the named factories in
     * [edu.wpi.first.wpilibj2.command.button.CommandGenericHID]'s subclasses for [edu.wpi.first.wpilibj2.command.button.CommandXboxController]/[ ] controllers or [edu.wpi.first.wpilibj2.command.button.CommandJoystick].
     */
    private fun configureBindings() {
        aacrn
            .bindings(
                bind(Button.START) { Kommand.resetPidgey() }, // bind(B, () -> setElevatorState(DEFAULT)),
                // bind(B, () -> align(CENTER).onlyWhile(pad::getAButton)),
                bind(Button.B) { Sequences.resetScore() }, // bind(B, () -> createPathfindingCmd(reefs.get(0))),
                bind(Button.A) { Kommand.setIntakeAlgae() }, // bind(A, () -> align(RIGHT)),
                bind(Button.Y) { Kommand.startCoralMotors() },
                bind(Button.X) { reverseIntake().onlyWhile(BooleanSupplier { aacrn.xButton }) },
                bind(Button.RIGHT_BUMPER) { fullScore(Direction.RIGHT) },
                bind(Button.LEFT_BUMPER) { fullScore(Direction.LEFT) },
            )

        calamityCow.bindings(bind(Button.A) { Kommand.offVision() })
        calamityCow.bindings(bind(Button.B) { Kommand.onVision() })
    }
}
