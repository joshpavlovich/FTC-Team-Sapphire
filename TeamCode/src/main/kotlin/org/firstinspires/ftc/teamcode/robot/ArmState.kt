package org.firstinspires.ftc.teamcode.robot

// Encoder Resolution for Arm 84 RPM Motor = ((((1+(46/17))) * (1+(46/17))) * (1+(46/11)) * 28)
// From https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-71-2-1-ratio-24mm-length-8mm-rex-shaft-84-rpm-3-3-5v-encoder/
// Ticks Per Revolution = (Motor Encoder Resolution / Diameter Millimeters)
// Encoder Resolution Formula -> ((((1+(46/17))) * (1+(46/17))) * (1+(46/11)) * 28) = 1992.6
private const val INTAKE_ARM_MOTOR_TICKS_PER_MM = (1992.6) / 96.0
private const val INTAKE_ARM_TRANSFER_POSITION = 2.3 * INTAKE_ARM_MOTOR_TICKS_PER_MM
private const val INTAKE_ARM_PICKUP_POSITION = 47.3 * INTAKE_ARM_MOTOR_TICKS_PER_MM
private const val INTAKE_ARM_LOW_CHAMBER_SCORING_POSITION = 31 * INTAKE_ARM_MOTOR_TICKS_PER_MM

sealed class ArmState(val position: Double) {
    data object Transfer : ArmState(INTAKE_ARM_TRANSFER_POSITION) // DEFAULT OR START STATE
    data object LowChamberScoring : ArmState(INTAKE_ARM_LOW_CHAMBER_SCORING_POSITION)
    data object IntakePickup : ArmState(INTAKE_ARM_PICKUP_POSITION)
    data class Moving(val currentPosition: Double) : ArmState(currentPosition)
}