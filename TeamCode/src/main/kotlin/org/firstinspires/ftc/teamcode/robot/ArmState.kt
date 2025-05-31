package org.firstinspires.ftc.teamcode.robot

// From https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-50-9-1-ratio-24mm-length-8mm-rex-shaft-117-rpm-3-3-5v-encoder/
// Ticks Per Revolution = (Motor Encoder Resolution / Diameter Millimeters)
// Encoder Resolution Formula	((((1+(46/17))) * (1+(46/17))) * (1+(46/17)) * 28) = 1,425.1
private const val INTAKE_ARM_MOTOR_TICKS_PER_MM = (1425.1) / 96.0
private const val INTAKE_ARM_TRANSFER_POSITION = 2.3 * INTAKE_ARM_MOTOR_TICKS_PER_MM
private const val INTAKE_ARM_PICKUP_POSITION = 47.3 * INTAKE_ARM_MOTOR_TICKS_PER_MM
private const val INTAKE_ARM_LOW_CHAMBER_SCORING_POSITION = 32 * INTAKE_ARM_MOTOR_TICKS_PER_MM

private const val RANGE_VARIANCE = 5

sealed class ArmState(val position: Int) {
    data object Transfer : ArmState(INTAKE_ARM_TRANSFER_POSITION.toInt()) // DEFAULT OR START STATE
    data object LowChamberScoring : ArmState(INTAKE_ARM_LOW_CHAMBER_SCORING_POSITION.toInt())
    data object IntakePickup : ArmState(INTAKE_ARM_PICKUP_POSITION.toInt())
    data class Moving(val currentPosition: Int) : ArmState(currentPosition)

    fun inRange(position: Int): Boolean {
        val lowerBound = this.position - RANGE_VARIANCE
        val upperBound = this.position + RANGE_VARIANCE
        return (position in lowerBound..upperBound)
    }
}