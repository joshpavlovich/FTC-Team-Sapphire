package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous

private const val DRIVE_SPEED: Double = 0.4

private const val SECONDS_TO_OBSERVATION_ZONE_FROM_START: Double = 1.65

@Autonomous(name = "LONG Autonomous", group = "Robot")
open class LongParkingMecanumAutonomous : ParkingMecanumAutonomous() {

    override fun getDriveSpeed(): Double {
        return DRIVE_SPEED
    }

    override fun getSecondsToObservationZoneFromStart(): Double {
        return SECONDS_TO_OBSERVATION_ZONE_FROM_START
    }

    override fun getSecondsToDelayFromStart(): Double = 0.0
}

@Autonomous(name = "DELAYED LONG Autonomous", group = "Robot")
class DelayedLongParkingMecanumAutonomous : LongParkingMecanumAutonomous() {
    override fun getSecondsToDelayFromStart(): Double = DELAY_SECONDS_PARKING_TO_OBSERVATION_ZONE
}