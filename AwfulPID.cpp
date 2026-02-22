#include "AwfulPID.h"
#include <Arduino.h>
#include "Debounce.h"
#include "GeneralFunctions.h"

AwfulPID::AwfulPID() {

}

void AwfulPID::setConfig(PIDConfiguration c) {
    cfg = c;
}

void AwfulPID::setParam(PIDParameters p) {
    param = p;
}

void AwfulPID::setManual(int _val) {
    iMV = _val;
}

int AwfulPID::update(byte ctrl, int _iPV, int _iSP) {    
    if (ctrl == PID_INIT) {
        // Clear and PID_INIT
        oCV = cfg.outMn;
        acc_ki = 0;      
        err_last = 0;
        stable = false;
        stableCycles = 0;

        // Prime timer to start on new cycle
        cycleTimer.update(cycleTimer.getState(),cfg.period_ms,cfg.period_ms);

    } else if (ctrl == PID_ENABLE) {
        // Use timer to create a continous clock frequency while PID_ENABLEd
        cycleTimer.update(!cycleTimer.getState(),cfg.period_ms,cfg.period_ms);
        
        // Recalc on every timer transition
        if (cycleTimer.getTransitionFlag()) {
            cycleTimer.resetTransitionFlag();

            // Push request to current
            iPV = _iPV;
            iSP = _iSP;
            
            // Calculations
            int err = calculateError();            
            if (err != 0) {
                int term_kp;
                int term_kd = 0;
                term_kp = (int) (param.kp * err); 
                if (param.ki > 0.00001) { acc_ki = acc_ki + (param.ki * err); }
                if (param.kd > 0.00001) { term_kd = param.kd * (err - err_last); }
                oCV = term_kp + acc_ki + term_kd;
            }

            // Monitor stability - Accumulate cycles
            if (abs(err) < cfg.stableTol) { stableCycles++; }
            else { stableCycles--; }

            // Set stable flag based on cycle accumulation
            if (stableCycles >= cfg.stablePeriodCount) {
                stableCycles = cfg.stablePeriodCount;
                stable = true;
            } else if (stableCycles < 1) {
                stableCycles = 0;
                stable = false;
            }

            // Hold on to this error for next calculation
            err_last = err;
        }    
    } else if (ctrl == PID_TIEBACK) {

    } else if (ctrl == PID_MANUAL) {
        oCV = iMV;
    }    
    oCV = limit(cfg.outMn, oCV, cfg.outMx);
    return oCV;
}

int AwfulPID::calculateError() {
    int e;
    if (cfg.reverseActing) { e = iPV - iSP; }
    else { e = iSP - iPV; }
    return e;
}

int AwfulPID::getError() {
    return err_last;
}

int AwfulPID::getCV() {
    return oCV;
}

bool AwfulPID::getStability() {
    return stable;
}