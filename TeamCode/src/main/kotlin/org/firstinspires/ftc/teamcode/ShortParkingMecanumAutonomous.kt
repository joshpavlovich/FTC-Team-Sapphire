package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled

private const val DRIVE_SPEED: Double = 0.4

private const val SECONDS_TO_OBSERVATION_ZONE_FROM_START: Double = 1.0

@Disabled
@Autonomous(name = "SHORT Autonomous", group = "Robot")
open class ShortParkingMecanumAutonomous : ParkingMecanumAutonomous() {

    override fun getDriveSpeed(): Double {
        return DRIVE_SPEED
    }

    override fun getSecondsToObservationZoneFromStart(): Double {
        return SECONDS_TO_OBSERVATION_ZONE_FROM_START
    }

    override fun getSecondsToDelayFromStart(): Double = 0.0
}

@Disabled
@Autonomous(name = "DELAYED SHORT Autonomous", group = "Robot")
class DelayedParkingMecanumAutonomous : ShortParkingMecanumAutonomous() {
    override fun getSecondsToDelayFromStart(): Double = DELAY_SECONDS_PARKING_TO_OBSERVATION_ZONE
}