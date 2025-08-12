package frc.robot

import edu.wpi.first.wpilibj.RobotBase
import java.util.function.Supplier

object Main {
    /** Main function Really, do NOT modify this file. We're serious, don't touch it.  */
    @JvmStatic
    fun main(args: Array<String>) {
        RobotBase.startRobot<Robot?>(Supplier { Robot() })
    }
}
