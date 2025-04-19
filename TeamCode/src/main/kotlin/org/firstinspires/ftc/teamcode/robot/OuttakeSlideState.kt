package org.firstinspires.ftc.teamcode.robot

// Encoder Resolution for Viper Slide 223 RPM Motor = ((((1+(46/11))) * (1+(46/11))) * 28)
// From https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-26-9-1-ratio-24mm-length-8mm-rex-shaft-223-rpm-3-3-5v-encoder/
// Ticks Per Revolution = (Motor Encoder Resolution / Diameter Millimeters)
// Encoder Resolution Formula -> ((((1+(46/11))) * (1+(46/11))) * 28) = 751.8
private const val SLIDE_LIFT_TICKS_PER_MM = (751.8) / 120
private const val SLIDE_LIFT_COLLAPSED = 0.0
private const val SLIDE_LIFT_LEVEL_ONE_ASCENT = 215.9 * SLIDE_LIFT_TICKS_PER_MM
// Distance in Millimeters for High Basket scoring position = high basket height in Millimeters * Viper Slide Lift Ticks Per Millimeter
private const val SLIDE_LIFT_SCORING_IN_HIGH_BASKET = 976.0 * SLIDE_LIFT_TICKS_PER_MM

sealed class OuttakeSlideState(val position: Double) {
    data object Collapsed : OuttakeSlideState(SLIDE_LIFT_COLLAPSED)
    data object LevelOneAscent : OuttakeSlideState(SLIDE_LIFT_LEVEL_ONE_ASCENT)
    data object ScoringInHighBasket : OuttakeSlideState(SLIDE_LIFT_SCORING_IN_HIGH_BASKET)
    data class Moving(val currentPosition: Double) : ArmState(currentPosition)
}