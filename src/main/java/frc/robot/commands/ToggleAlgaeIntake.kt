package frc.robot.commands

import frc.robot.commands.Kommand.cmd
import frc.robot.subsystems.Elevator
import frc.robot.utils.RobotParameters.AlgaeManipulatorParameters.algaeCounter
import frc.robot.utils.RobotParameters.AlgaeManipulatorParameters.algaeIntaking
import frc.robot.utils.RobotParameters.AlgaeManipulatorParameters.algaePivotState
import frc.robot.utils.RobotParameters.CoralManipulatorParameters.coralState
import frc.robot.utils.RobotParameters.ElevatorParameters.elevatorToBeSetState
import frc.robot.utils.emu.AlgaeCounter
import frc.robot.utils.emu.AlgaePivotState
import frc.robot.utils.emu.CoralState
import frc.robot.utils.emu.ElevatorState

val toggleIntakeAlgae =
    cmd {
        when (algaeCounter) {
            AlgaeCounter.INTAKE ->
                when (elevatorToBeSetState) {
                    ElevatorState.L2 -> {
                        algaePivotState = AlgaePivotState.DOWN
                        coralState = CoralState.ALGAE_INTAKE
                        Elevator.state = ElevatorState.ALGAE_LOW
                        algaeIntaking = true
                    }
                    ElevatorState.L3 -> {
                        algaePivotState = AlgaePivotState.DOWN
                        coralState = CoralState.ALGAE_INTAKE
                        Elevator.state = ElevatorState.ALGAE_HIGH
                        algaeIntaking = true
                    }
                    else -> {
                        algaeCounter = AlgaeCounter.DEFAULT
                    }
                }

            AlgaeCounter.DEFAULT -> {
                algaePivotState = AlgaePivotState.UP
                coralState = CoralState.CORAL_INTAKE
                algaeIntaking = false
                Elevator.state = ElevatorState.DEFAULT
            }
        }
        algaeCounter = algaeCounter.next
    }
