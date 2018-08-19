clearscreen.

print "start".

function doStart {
    parameter wantedApoapsos.
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



    UNTIL SHIP:APOAPSIS > wantedApoapsos {
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
}

function findNextNode{
    return nextnode.
}

function executeNode {
    parameter nd.
    print "Node in: " + round(nd:eta) + ", DeltaV: " + round(nd:deltav:mag).

    //calculate ship's max acceleration
    set max_acc to ship:maxthrust/ship:mass.
    set burn_duration to nd:deltav:mag/max_acc.
    print "Crude Estimated burn duration: " + round(burn_duration) + "s".
    wait until nd:eta <= (burn_duration/2 + 60).

    set np to nd:deltav. //points to node, don't care about the roll direction.
    lock steering to np.

    //now we need to wait until the burn vector and ship's facing are aligned
    wait until vang(np, ship:facing:vector) < 0.25.

    //the ship is facing the right direction, let's wait for our burn time
    wait until nd:eta <= (burn_duration/2).

    set thrott to 0.


    set done to False.
    //initial deltav
    set dv0 to nd:deltav.
    until done
    {
        //recalculate current max_acceleration, as it changes while we burn through fuel
        set max_acc to ship:maxthrust/ship:mass.

        //throttle is 100% until there is less than 1 second of time left to burn
        //when there is less than 1 second - decrease the throttle linearly
        set thrott to min(nd:deltav:mag/max_acc, 1).

        //here's the tricky part, we need to cut the throttle as soon as our nd:deltav and initial deltav start facing opposite directions
        //this check is done via checking the dot product of those 2 vectors
        if vdot(dv0, nd:deltav) < 0
        {
            print "End burn, remain dv " + round(nd:deltav:mag,1) + "m/s, vdot: " + round(vdot(dv0, nd:deltav),1).
            lock throttle to 0.
            break.
        }

        //we have very little left to burn, less then 0.1m/s
        if nd:deltav:mag < 0.1
        {
            print "Finalizing burn, remain dv " + round(nd:deltav:mag,1) + "m/s, vdot: " + round(vdot(dv0, nd:deltav),1).
            //we burn slowly until our node vector starts to drift significantly from initial vector
            //this usually means we are on point
            wait until vdot(dv0, nd:deltav) < 0.5.

            lock throttle to 0.
            print "End burn, remain dv " + round(nd:deltav:mag,1) + "m/s, vdot: " + round(vdot(dv0, nd:deltav),1).
            set done to True.
        }
    }
    unlock steering.
    unlock throttle.


}

function main {
    doStart(100000).

    wait 60.
    set nd to findNextNode().
    executeNode(nd).
    remove nd.
    SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
}


main().
