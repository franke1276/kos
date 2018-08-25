clearscreen.

print "start".

function doStart {
    parameter wantedApoapsos, planet, targetDirection.
    SET g TO planet:MU / planet:RADIUS^2.
    LOCK accvec TO SHIP:SENSORS:ACC - SHIP:SENSORS:GRAV.
    LOCK gforce TO accvec:MAG / g.

    SET Kp TO 0.003.
    SET Ki TO 0.005.
    SET Kd TO 0.0005.

    SET PID TO PIDLOOP(Kp, Kp, Kd).
    SET PID:SETPOINT TO 1.7.
    SET thrott TO 1.
    LOCK THROTTLE TO thrott.

    STAGE.
    wait until STAGE:READY.
    wait 0.5.
    STAGE.

    WHEN MAXTHRUST = 0 THEN {
        IF STAGE:NUMBER > 0
            PRESERVE.
        
    }.

    //lock angle to 90.0909 - 0.000609091 * alt:radar - 4.54545E-8 * alt:radar^2.
    //lock angle to 88.963 - 1.03287 * alt:radar^0.409511.
    // lock angle to (17 * alt:radar^2)/1600000000 - (157 * alt:radar/80000) + 90.
    lock angle to 90 -  alt:radar * 0.003.
    lock steering to HEADING(targetDirection, angle).
    when angle < 45 then {
        unlock steering.
        lock steering to ship:prograde.
    }

    UNTIL SHIP:APOAPSIS > wantedApoapsos {
        SET thrott TO thrott + PID:UPDATE(TIME:SECONDS, gforce).
        print "alt: " +  ROUND(alt:radar,0) AT (0,17).

        WAIT 0.001.
    }
    SET thrott TO 0.
    unlock steering.
    unlock THROTTLE.
    SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
    WAIT 2.
    stage.
}


function improve {
    parameter node, scoreFunction, step.
     local canditates to list().
    FROM {local index is 0.} UNTIL index >= node:length STEP {set index to index + 1.} DO {
         local incNode to node:copy().
        set incNode[index] to incNode[index] + step.
         local decNode to node:copy().
        set decNode[index] to decNode[index] - step.
        canditates:add(incNode).
        canditates:add(decNode).
    }
     local bestCanditate to node.
     local scoreBest to scoreFunction(bestCanditate).
    for candidate in canditates {
        local scoreCanditate to scoreFunction(candidate).
        if (scoreCanditate < scoreBest) {
            set bestCanditate to candidate.
            set scoreBest to scoreFunction(bestCanditate).
        }
    }
    return bestCanditate.
}

function eccentricityScore {
    parameter nodeList.
    
    local node to node(time:seconds + eta:APOAPSIS, 0, 0, nodeList[0]).
    add node.
    local score to node:orbit:eccentricity.
    remove node.
    return score.
}

function findNextBestNode {
    parameter startNode, scoreFunction.
    local candidate to startNode.
    for step in list(100, 10, 1) {
        until false {
            local oldScore to scoreFunction(candidate).
            local newCandidate to improve(candidate, eccentricityScore@, step).
            local newScore to scoreFunction(newCandidate).
            if oldScore < newScore  {
                break.
            }
            set candidate to newCandidate.
        }
    }
    return candidate.
}

function calculateBurnTime {
    parameter nd.
    local dV is nd:deltav:mag.
    local g0 is 9.80665.
    local isp is 0.

    list engines in myEngines.
    for en in myEngines {
        if en:ignition and not en:flameout {
            set isp to isp + (en:isp * (en:maxthrust / ship:maxthrust)).
        }
    }
    local mf is ship:mass / constant():e^(dV / (isp * g0)).
    local fuelFlow is ship:maxthrust / (isp * g0).
    local t is (ship:mass - mf) / fuelFlow.
    return t.
}

function executeNode {
    parameter nd.
    add nd.
    print "Node in: " + round(nd:eta) + ", DeltaV: " + round(nd:deltav:mag).

    
    
    local burn_duration to calculateBurnTime(nd).
    print "Crude Estimated burn duration: " + round(burn_duration) + "s".
    
    set np to nd:deltav. 
    lock steering to nd:deltav.

    //now we need to wait until the burn vector and ship's facing are aligned
    //wait until vang(np, ship:facing:vector) < 0.25.

    set thrott to 0.
    LOCK THROTTLE TO thrott.
    
    wait until nd:eta <= (burn_duration/2).
    set thrott to 1.
    
    set done to False.
    
    set dv0 to nd:deltav.
    until done {
        //recalculate current max_acceleration, as it changes while we burn through fuel
        set max_acc to ship:maxthrust/ship:mass.

        //throttle is 100% until there is less than 1 second of time left to burn
        //when there is less than 1 second - decrease the throttle linearly
        set thrott to min(nd:deltav:mag/max_acc, 1).

        //here's the tricky part, we need to cut the throttle as soon as our nd:deltav and initial deltav start facing opposite directions
        //this check is done via checking the dot product of those 2 vectors
        if vdot(dv0, nd:deltav) < 0 {
            print "End burn, remain dv " + round(nd:deltav:mag,1) + "m/s, vdot: " + round(vdot(dv0, nd:deltav),1).
            set thrott to 0.
            break.
        }

        //we have very little left to burn, less then 0.1m/s
        if nd:deltav:mag < 0.1 {
            print "Finalizing burn, remain dv " + round(nd:deltav:mag,1) + "m/s, vdot: " + round(vdot(dv0, nd:deltav),1).
            //we burn slowly until our node vector starts to drift significantly from initial vector
            //this usually means we are on point
            wait until vdot(dv0, nd:deltav) < 0.5.

            set thrott to 0.
            print "End burn, remain dv " + round(nd:deltav:mag,1) + "m/s, vdot: " + round(vdot(dv0, nd:deltav),1).
            set done to True.
        }
    }
    unlock steering.
    unlock throttle.
    
}

function doCircularize {
    set node to findNextBestNode(list(0), eccentricityScore@).
    local nd to node(time:seconds + eta:APOAPSIS, 0, 0, node[0]).
    //  add nd.
    executeNode(nd).
}

function main {
    doStart(100000, KERBIN, 90).
   
    doCircularize().
}


main().
