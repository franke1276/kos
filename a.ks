clearscreen.

print "start".
SET g TO KERBIN:MU / KERBIN:RADIUS^2.
LOCK accvec TO SHIP:SENSORS:ACC - SHIP:SENSORS:GRAV.
LOCK gforce TO accvec:MAG / g.

SET Kp TO 0.003.
SET Ki TO 0.005.
SET Kd TO 0.0005.

SET PID TO PIDLOOP(Kp, Kp, Kd).
SET PID:SETPOINT TO 1.8.
SET thrott TO 1.
LOCK THROTTLE TO thrott.

// lock targetPitch to 88.963 - 1.03287 * alt:radar^0.409511.
// set targetDirection to 90.
// lock steering to heading(targetDirection, targetPitch).
set MYSTEER to heading(90,90).
lock steering to MYSTEER.

WHEN MAXTHRUST = 0 THEN {
    PRINT "Staging".
    STAGE.
    IF STAGE:NUMBER > 0
        PRESERVE.
    
}.



UNTIL SHIP:APOAPSIS > 100000 {
    SET thrott TO thrott + PID:UPDATE(TIME:SECONDS, gforce).
    print "alt: " +  ROUND(alt:radar,0) AT (0,17).
    
     IF SHIP:VELOCITY:SURFACE:MAG < 100 {
        //This sets our steering 90 degrees up and yawed to the compass
        //heading of 90 degrees (east)
        SET MYSTEER TO HEADING(90,90).

    //Once we pass 100m/s, we want to pitch down ten degrees
    } ELSE IF SHIP:VELOCITY:SURFACE:MAG >= 100 AND SHIP:VELOCITY:SURFACE:MAG < 200 {
        SET MYSTEER TO HEADING(90,80).
        PRINT "Pitching to 80 degrees" AT(0,15).
        PRINT ROUND(SHIP:APOAPSIS,0) AT (0,16).

    //Each successive IF statement checks to see if our velocity
    //is within a 100m/s block and adjusts our heading down another
    //ten degrees if so
    } ELSE IF SHIP:VELOCITY:SURFACE:MAG >= 200 AND SHIP:VELOCITY:SURFACE:MAG < 300 {
        SET MYSTEER TO HEADING(90,75).
        PRINT "Pitching to 70 degrees" AT(0,15).
        PRINT ROUND(SHIP:APOAPSIS,0) AT (0,16).

    } ELSE IF SHIP:VELOCITY:SURFACE:MAG >= 300 AND SHIP:VELOCITY:SURFACE:MAG < 400 {
        SET MYSTEER TO HEADING(90,70).
        PRINT "Pitching to 60 degrees" AT(0,15).
        PRINT ROUND(SHIP:APOAPSIS,0) AT (0,16).

    } ELSE IF SHIP:VELOCITY:SURFACE:MAG >= 400 AND SHIP:VELOCITY:SURFACE:MAG < 500 {
        SET MYSTEER TO HEADING(90,65).
        PRINT "Pitching to 45 degrees" AT(0,15).
        PRINT "APOAPSIS: " + ROUND(SHIP:APOAPSIS,0) AT (0,16).

    } ELSE IF SHIP:VELOCITY:SURFACE:MAG >= 700 AND SHIP:VELOCITY:SURFACE:MAG < 850 {
        SET MYSTEER TO HEADING(90,60).
        PRINT "Pitching to 45 degrees" AT(0,15).
        PRINT "APOAPSIS: " + ROUND(SHIP:APOAPSIS,0) AT (0,16).

    } ELSE IF SHIP:VELOCITY:SURFACE:MAG >= 850 AND SHIP:VELOCITY:SURFACE:MAG < 950 {
        SET MYSTEER TO HEADING(90,45).
        PRINT "Pitching to 45 degrees" AT(0,15).
        PRINT "APOAPSIS: " + ROUND(SHIP:APOAPSIS,0) AT (0,16).

    } ELSE IF SHIP:VELOCITY:SURFACE:MAG >= 1000  {
        SET MYSTEER TO SHIP:PROGRADE.
        PRINT "Pitching to SHIP:PROGRADE degrees" AT(0,15).
        PRINT "APOAPSIS: " + ROUND(SHIP:APOAPSIS,0) AT (0,16).

    }.

    WAIT 0.001.
}
SET thrott TO 0.
unlock steering.
WAIT 2.
stage.
SET BURNING TO FALSE.
SET HEAD TO SHIP:PROGRADE.
LOCK STEERING TO HEAD.
SET ROW_STATE TO 20.
set lastEta to 0.
set timetoburn to 60.
UNTIL SHIP:PERIAPSIS > 100000 {
	// Hold the ship pointing prograde
	SET HEAD TO SHIP:PROGRADE.

	IF BURNING {
		PRINT "Burning into stable orbit"  AT (0,ROW_STATE).
		PRINT "  AP = "+ROUND(SHIP:APOAPSIS,0)+"      " AT (0,ROW_STATE+1).
		PRINT "  PE = "+ROUND(SHIP:PERIAPSIS,0)+"      " AT (0,ROW_STATE+2).
        PRINT "  ETA = "+ROUND(ETA:APOAPSIS,0)+"      " AT (0,ROW_STATE+3).
        if ETA:APOAPSIS > lastEta {
            PRINT "stop burning"  AT (0,ROW_STATE + 4) .
            LOCK THROTTLE TO 0.    
            SET BURNING TO FALSE.
            set timetoburn to timetoburn - 2.
        }.
        set lastEta to ETA:APOAPSIS.
	} ELSE IF ETA:APOAPSIS < timetoburn {
		PRINT "Burning.".
		PRINT " ETA = NOW               " AT (0,ROW_STATE+3).
		LOCK THROTTLE TO 1.
		SET BURNING TO TRUE.
        set lastEta to ETA:APOAPSIS.
	} ELSE {
		PRINT "Waiting until apoapsis..."  AT (0,ROW_STATE).
		PRINT "  AP = "+ROUND(SHIP:APOAPSIS,0)+"      " AT (0,ROW_STATE+1).
		PRINT "  PE = "+ROUND(SHIP:PERIAPSIS,0)+"      " AT (0,ROW_STATE+2).
		PRINT " ETA = "+ROUND(ETA:APOAPSIS,0)+"      " AT (0,ROW_STATE+3).
	}.
}.

unlock steering.
unlock throttle.
print "end".