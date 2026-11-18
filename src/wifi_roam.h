#pragma once
// Move to a clearly stronger mesh node while connected. Pure logic, so
// tests/wifi_roam_test.cpp can drive it on the host.
//
// The join already picks the strongest node (wifi_prefer_strongest_ap() in main.cpp), but
// once connected the ESP32 never looks again. After a mesh restart the far nodes can come
// back first, and the radar then stays on a weak link until it reboots. This class decides
// WHEN to look (a scan of our SSID) and WHETHER the result is worth a move; adsb_task does
// the scan and the move between fetches.
//
// A move is a disconnect + reconnect with the saved config, never a join by BSSID: the
// saved config already scans every channel and sorts by signal, so the driver picks the
// node, and nothing is left pinned to a node that may later go away.
//
// When to look: WIFI_ROAM_FIRST_CHECK_MS after WiFi comes up, then every
// WIFI_ROAM_PERIOD_MS; sooner (but no more often than WIFI_ROAM_WEAK_GAP_MS) when the
// average of the last 3 feed polls' RSSI is below WIFI_ROAM_WEAK_DBM. Never within
// WIFI_ROAM_MOVE_GAP_MS of a move.
// Whether to move: the strongest OTHER node is at least WIFI_ROAM_MARGIN_DB stronger than
// ours and at least WIFI_ROAM_MIN_DBM. If a move rejoins the same node, the period doubles
// (up to WIFI_ROAM_MAX_PERIOD_MS) so a scan the driver disagrees with can't make it bounce.
//
// A network with one access point (a single router, a phone hotspot) keeps being checked,
// since a node can be added or come back later (after a mesh restart the far node may be the
// only one up at the first check), but less often: after WIFI_ROAM_ALONE_CHECKS checks in a
// row hear no other node, the period goes to 20, then 40 min, and back to 10 as soon as
// another node is heard or WiFi rejoins. While no other node is heard the weak-signal
// trigger is off (on a weak single access point it would scan every 2 min for nothing), and
// the all-channel scan runs at most every WIFI_ROAM_WIDE_GAP_MS.
#include <stdint.h>
#include <string.h>
#include "config.h"

class WifiRoam {
public:
    struct Seen { uint8_t bssid[6]; int rssi; };        // one scan result for our SSID
    enum Due     { NOT_DUE, DUE_PERIODIC, DUE_WEAK };
    enum Verdict { STAY_NO_OTHER, STAY_TOO_WEAK, STAY_NOT_BETTER, MOVE };
    enum Rejoin  { JOINED, MOVED, SAME_NODE };

    // WiFi came up on `bssid`. Returns what that means for a pending move.
    Rejoin onConnected(const uint8_t bssid[6], uint32_t nowMs) {
        _up = true;
        _nSamples = 0;
        _alone = 0;                      // a rejoin may be on a different network or node set
        Rejoin r = JOINED;
        if (_moving) {
            _moving = false;
            if (memcmp(bssid, _movedFrom, 6) == 0) {
                r = SAME_NODE;
                _periodMs = (_periodMs * 2 > WIFI_ROAM_MAX_PERIOD_MS) ? WIFI_ROAM_MAX_PERIOD_MS
                                                                      : _periodMs * 2;
            } else {
                r = MOVED;
                _periodMs = WIFI_ROAM_PERIOD_MS;
            }
            _nextMs = nowMs + (_periodMs > WIFI_ROAM_MOVE_GAP_MS ? _periodMs : WIFI_ROAM_MOVE_GAP_MS);
        } else {
            _nextMs = nowMs + WIFI_ROAM_FIRST_CHECK_MS;
        }
        _lastCheckMs = nowMs;            // the weak trigger also waits its gap after a join
        return r;
    }

    // WiFi went down (not one of our moves): nothing to check until it is back.
    void onDisconnected() { _up = false; }

    // RSSI after each feed poll; the last 3 feed the weak-signal trigger.
    void onPollRssi(int rssi) {
        _samples[_nSamples % 3] = rssi;
        if (_nSamples < 255) _nSamples++;
    }

    bool weak() const {
        if (_nSamples < 3) return false;
        return (_samples[0] + _samples[1] + _samples[2]) < 3 * WIFI_ROAM_WEAK_DBM;
    }

    // The average weak() looks at, for the log line (a check's own reading can differ from it
    // by 10 dB or more near the edge). False until there are 3 samples.
    bool averageRssi(float* avg) const {
        if (_nSamples < 3) return false;
        *avg = (_samples[0] + _samples[1] + _samples[2]) / 3.0f;
        return true;
    }

    Due due(uint32_t nowMs) const {
        if (!_up) return NOT_DUE;                   // down, or in the middle of a move
        if (_movedOnce && (int32_t)(nowMs - _lastMoveMs) < (int32_t)WIFI_ROAM_MOVE_GAP_MS)
            return NOT_DUE;
        if ((int32_t)(nowMs - _nextMs) >= 0) return DUE_PERIODIC;
        if (_alone == 0 && weak() &&
            (int32_t)(nowMs - _lastCheckMs) >= (int32_t)WIFI_ROAM_WEAK_GAP_MS)
            return DUE_WEAK;
        return NOT_DUE;
    }

    // A one-channel scan found no other node: worth one all-channel scan only on a weak link
    // (the mesh may have split its channels), and at most every WIFI_ROAM_WIDE_GAP_MS.
    // True means the caller runs that scan now, so the time is recorded here.
    bool wantWideScan(const Seen* seen, int n, const uint8_t cur[6], uint32_t nowMs) {
        if (!weak() || bestOther(seen, n, cur) >= 0) return false;
        if (_wideOnce && (int32_t)(nowMs - _lastWideMs) < (int32_t)WIFI_ROAM_WIDE_GAP_MS)
            return false;
        _wideOnce = true;
        _lastWideMs = nowMs;
        return true;
    }

    // The scan is done: decide, and restart the schedule. `curRssi` is WiFi.RSSI(); the
    // scan's own reading of our node is used instead when it lists it, so both numbers come
    // from the same scan. `*best` = index of the strongest other node, or -1.
    Verdict decide(const uint8_t cur[6], int curRssi, const Seen* seen, int n, uint32_t nowMs,
                   int* best, int* hereRssi) {
        _lastCheckMs = nowMs;
        for (int i = 0; i < n; i++)
            if (memcmp(seen[i].bssid, cur, 6) == 0) { curRssi = seen[i].rssi; break; }
        if (hereRssi) *hereRssi = curRssi;
        const int b = bestOther(seen, n, cur);
        if (best) *best = b;
        if (b < 0) { if (_alone < 255) _alone++; } else _alone = 0;
        _nextMs = nowMs + nextPeriodMs();
        if (b < 0) return STAY_NO_OTHER;
        if (seen[b].rssi < WIFI_ROAM_MIN_DBM) return STAY_TOO_WEAK;
        if (seen[b].rssi - curRssi < WIFI_ROAM_MARGIN_DB) return STAY_NOT_BETTER;
        return MOVE;
    }

    // decide() said MOVE but the radar is in use (touched recently, or a detail card open):
    // look again WIFI_ROAM_DEFER_MS later instead, with a fresh scan.
    void deferMove(uint32_t nowMs) { _nextMs = nowMs + WIFI_ROAM_DEFER_MS; }

    // The scan itself failed: try again after the normal period, not on the next pass.
    void scanFailed(uint32_t nowMs) {
        _lastCheckMs = nowMs;
        _nextMs = nowMs + nextPeriodMs();
    }

    // adsb_task is about to disconnect and reconnect.
    void onMoveStarted(const uint8_t from[6], uint32_t nowMs) {
        memcpy(_movedFrom, from, 6);
        _moving = true;
        _movedOnce = true;
        _lastMoveMs = nowMs;
        _up = false;
    }

    uint32_t periodMs() const { return _periodMs; }
    uint32_t nextPeriodMs() const {      // the wait after a check: the longer of the two slow-downs
        uint32_t p = WIFI_ROAM_PERIOD_MS;
        for (int i = WIFI_ROAM_ALONE_CHECKS; i <= _alone && p < WIFI_ROAM_MAX_PERIOD_MS; i++) p *= 2;
        if (p > WIFI_ROAM_MAX_PERIOD_MS) p = WIFI_ROAM_MAX_PERIOD_MS;
        return p > _periodMs ? p : _periodMs;
    }
    int aloneChecks() const { return _alone; }

private:
    static int bestOther(const Seen* seen, int n, const uint8_t cur[6]) {
        int b = -1;
        for (int i = 0; i < n; i++) {
            if (memcmp(seen[i].bssid, cur, 6) == 0) continue;
            if (b < 0 || seen[i].rssi > seen[b].rssi) b = i;
        }
        return b;
    }

    bool     _up = false;
    bool     _moving = false;
    bool     _movedOnce = false;
    bool     _wideOnce = false;
    uint8_t  _alone = 0;                 // checks in a row that heard no other node
    uint32_t _lastWideMs = 0;
    uint8_t  _movedFrom[6] = {0};
    uint32_t _nextMs = 0;
    uint32_t _lastCheckMs = 0;
    uint32_t _lastMoveMs = 0;
    uint32_t _periodMs = WIFI_ROAM_PERIOD_MS;
    int      _samples[3] = {0, 0, 0};
    uint8_t  _nSamples = 0;
};
