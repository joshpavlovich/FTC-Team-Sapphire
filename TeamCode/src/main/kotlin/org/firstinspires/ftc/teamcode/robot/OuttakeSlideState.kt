package org.firstinspires.ftc.teamcode.robot

// Encoder Resolution for Viper Slide 435 RPM Motor = ((((1+(46/17))) * (1+(46/17))) * 28)
// From https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-8mm-rex-shaft-435-rpm-3-3-5v-encoder/
// Ticks Per Revolution = (Motor Encoder Resolution / Diameter Millimeters)
// Encoder Resolution Formula -> ((((1+(46/17))) * (1+(46/17))) * 28) = 384.54
private const val SLIDE_LIFT_TICKS_PER_MM = (384.54) / 120
private const val SLIDE_LIFT_COLLAPSED = 0.0
private const val SLIDE_LIFT_LEVEL_ONE_ASCENT = 215.9 * SLIDE_LIFT_TICKS_PER_MM
// Distance in Millimeters for High Basket scoring position = high basket height in Millimeters * Viper Slide Lift Ticks Per Millimeter
private const val SLIDE_LIFT_SCORING_IN_HIGH_BASKET = 976.0 * SLIDE_LIFT_TICKS_PER_MM

private const val RANGE_VARIANCE = 5

sealed class OuttakeSlideState(val position: Int) {
    data object Collapsed : OuttakeSlideState(SLIDE_LIFT_COLLAPSED.toInt())
    data object LevelOneAscent : OuttakeSlideState(SLIDE_LIFT_LEVEL_ONE_ASCENT.toInt())
    data object ScoringInHighBasket : OuttakeSlideState(SLIDE_LIFT_SCORING_IN_HIGH_BASKET.toInt())
    data class Moving(val currentPosition: Int) : OuttakeSlideState(currentPosition)

    fun inRange(position: Int): Boolean {
        val lowerBound = this.position - RANGE_VARIANCE
        val upperBound = this.position + RANGE_VARIANCE
        return (position in lowerBound..upperBound)
    }
}