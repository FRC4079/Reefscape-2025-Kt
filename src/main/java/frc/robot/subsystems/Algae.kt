package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.utils.RobotParameters.AlgaeManipulatorParameters
import frc.robot.utils.RobotParameters.AlgaeManipulatorParameters.algaeCounter
import frc.robot.utils.RobotParameters.AlgaeManipulatorParameters.algaeIntaking
import frc.robot.utils.RobotParameters.AlgaeManipulatorParameters.algaePivotState
import frc.robot.utils.RobotParameters.MotorParameters.ALGAE_PIVOT_MOTOR_ID
import frc.robot.utils.emu.AlgaePivotState
import frc.robot.utils.pingu.AlertPingu.add
import frc.robot.utils.pingu.LogPingu.log
import frc.robot.utils.pingu.LogPingu.logs

/**
 * The PivotSubsystem class is a subsystem that interfaces with the arm system to provide control
 * over the arm motors. This subsystem is a Singleton, meaning that only one instance of this class
 * is created and shared across the entire robot code.
 */
object Algae : SubsystemBase() {
    /** Creates a new end effector.  */
    private val algaePivotMotor: TalonFX

    //    private final TalonFX algaeIntakeMotor;
    private val voltageOut: VoltageOut
    private val voltagePos: PositionVoltage

    /**
     * Creates a new instance of this armSubsystem. This constructor is private since this class is a
     * Singleton. Code should use the [.getInstance] method to get the singleton instance.
     */
    init {
        algaePivotMotor = TalonFX(ALGAE_PIVOT_MOTOR_ID)

        //        algaeIntakeMotor = new TalonFX(ALGAE_INTAKE_MOTOR_ID);
        val algaePivotConfiguration = TalonFXConfiguration()

        //        TalonFXConfiguration algaeIntakeConfiguration = new TalonFXConfiguration();
        algaePivotConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake

        //        algaeIntakeConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        algaePivotConfiguration.Slot0.kP = AlgaeManipulatorParameters.ALGAE_PINGU.p
        algaePivotConfiguration.Slot0.kI = AlgaeManipulatorParameters.ALGAE_PINGU.i
        algaePivotConfiguration.Slot0.kD = AlgaeManipulatorParameters.ALGAE_PINGU.d

        algaePivotConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive

        //        algaeIntakeConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        algaePivotMotor.configurator.apply(algaePivotConfiguration)

        //        algaeIntakeMotor.getConfigurator().apply(algaeIntakeConfiguration);
        algaePivotConfiguration.CurrentLimits.SupplyCurrentLimit = 30.0
        algaePivotConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
        algaePivotConfiguration.CurrentLimits.StatorCurrentLimit = 30.0
        algaePivotConfiguration.CurrentLimits.StatorCurrentLimitEnable = true

        //        algaeIntakeConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
//        algaeIntakeConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
//        algaeIntakeConfiguration.CurrentLimits.StatorCurrentLimit = 30;
//        algaeIntakeConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        algaePivotConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0
        algaePivotConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = false
        algaePivotConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0
        algaePivotConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = false

        algaePivotMotor.configurator.apply(algaePivotConfiguration)

        //        algaeIntakeMotor.getConfigurator().apply(algaeIntakeConfiguration);

        //    algaeManipulatorMotorConfiguration.MotorOutput.Inverted =
        // InvertedValue.Clockwise_Positive;
        //
        //    algaeManipulatorMotorConfiguration.SoftwareLimitSwitch =
        // algaeManipulatorMotorSoftLimitConfig;
        voltageOut = VoltageOut(0.0)
        voltagePos = PositionVoltage(0.0)

        algaePivotMotor.setPosition(0.0)

        add(algaePivotMotor, "algae pivot")
        //        AlertPingu.add(algaeIntakeMotor, "algae intake");
    }

    // This method will be called once per scheduler run
    override fun periodic() {
        setPivotPos(algaePivotState)

        //        setIntakeSpeed(algaePivotState);
        logs(
            Runnable {
                log("Algae/Algae Pivot Motor Position", this.pivotPosValue)
                log("Algae/Algae State", algaePivotState.toString())
                log("Algae/IsAlgaeIntaking", algaeIntaking)
                log("Algae/Algae counter", algaeCounter.toString())
                log(
                    "Algae/Disconnected algaeManipulatorMotor " + algaePivotMotor.deviceID,
                    algaePivotMotor.isConnected,
                )
                log(
                    "Algae/Algae Pivot Stator Current",
                    algaePivotMotor.statorCurrent.valueAsDouble,
                )
                log(
                    "Algae/Algae Pivot Supply Current",
                    algaePivotMotor.supplyCurrent.valueAsDouble,
                )
                log(
                    "Algae/Algae Pivot Stall Current",
                    algaePivotMotor.motorStallCurrent.valueAsDouble,
                )
            },
        )
    }

    /**
     * Sets the pivot state *
     *
     * @param state the state to set the algae pivot
     */
    fun setPivotPos(state: AlgaePivotState) {
        algaePivotMotor.setControl(voltagePos.withPosition(state.pos))
    }

    val pivotPosValue: Double
        /**
         * Get the position of the end effector motor
         *
         * @return double, the position of the end effector motor
         */
        get() =
            algaePivotMotor.position
                .valueAsDouble //    public void setIntakeSpeed(AlgaePivotState state) {
    //        voltageOut.Output = state.intakeSpeed;
    //        algaeIntakeMotor.setControl(voltageOut);
    //    }
}
