// Host tests for the mesh-node move logic (src/wifi_roam.h): when a connected radar looks
// again, which scan results are worth a move, the back-off when a move rejoins the same
// node, and the cases seen on real hardware (dBm figures from the radar logs).
#include "wifi_roam.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

static const uint32_t MIN = 60UL * 1000UL;

// Mesh nodes (made-up BSSIDs, one per node).
static const uint8_t NEAR[6]     = {0x02, 0, 0, 0, 0, 1};
static const uint8_t FAR[6]      = {0x02, 0, 0, 0, 0, 2};
static const uint8_t FARTHEST[6] = {0x02, 0, 0, 0, 0, 3};

static WifiRoam::Seen seen(const uint8_t b[6], int rssi) {
    WifiRoam::Seen s;
    for (int i = 0; i < 6; i++) s.bssid[i] = b[i];
    s.rssi = rssi;
    return s;
}

// One check on `cur` (WiFi.RSSI() = curRssi) against the listed scan results.
static WifiRoam::Verdict check(WifiRoam& r, const uint8_t cur[6], int curRssi,
                               const WifiRoam::Seen* s, int n, uint32_t now, int* best = nullptr) {
    int b = -2, here = 0;
    WifiRoam::Verdict v = r.decide(cur, curRssi, s, n, now, &b, &here);
    if (best) *best = b;
    return v;
}

static void schedule(uint32_t t0) {
    WifiRoam r;
    assert(r.due(t0) == WifiRoam::NOT_DUE);                    // not connected yet
    assert(r.onConnected(NEAR, t0) == WifiRoam::JOINED);
    assert(r.due(t0 + MIN) == WifiRoam::NOT_DUE);
    assert(r.due(t0 + 3 * MIN - 1) == WifiRoam::NOT_DUE);
    assert(r.due(t0 + 3 * MIN) == WifiRoam::DUE_PERIODIC);     // first look 3 min after joining
    WifiRoam::Seen s[] = {seen(NEAR, -39)};
    assert(check(r, NEAR, -39, s, 1, t0 + 3 * MIN) == WifiRoam::STAY_NO_OTHER);
    assert(r.due(t0 + 13 * MIN - 1) == WifiRoam::NOT_DUE);
    assert(r.due(t0 + 13 * MIN) == WifiRoam::DUE_PERIODIC);    // then every 10 min
    r.onDisconnected();
    assert(r.due(t0 + 20 * MIN) == WifiRoam::NOT_DUE);         // nothing to check while down
    assert(r.onConnected(NEAR, t0 + 21 * MIN) == WifiRoam::JOINED);
    assert(r.due(t0 + 23 * MIN) == WifiRoam::NOT_DUE);
    assert(r.due(t0 + 24 * MIN) == WifiRoam::DUE_PERIODIC);    // a rejoin restarts at 3 min
}

int main() {
    // ---- the schedule, also across millis() wrapping ----
    schedule(1000);
    schedule(0xFFFFFFFFu - 2 * MIN);           // the first check's deadline wraps, "now" not yet

    // ---- verdicts ----
    {
        WifiRoam r;
        r.onConnected(NEAR, 0);
        int best;
        WifiRoam::Seen none[] = {seen(NEAR, -40)};
        assert(check(r, NEAR, -40, none, 1, 0, &best) == WifiRoam::STAY_NO_OTHER && best == -1);
        assert(check(r, NEAR, -40, nullptr, 0, 0, &best) == WifiRoam::STAY_NO_OTHER && best == -1);

        WifiRoam::Seen plus9[] = {seen(FAR, -59)};
        assert(check(r, NEAR, -68, plus9, 1, 0) == WifiRoam::STAY_NOT_BETTER);   // +9 dB
        WifiRoam::Seen plus10[] = {seen(FAR, -64)};
        assert(check(r, NEAR, -74, plus10, 1, 0, &best) == WifiRoam::MOVE && best == 0);  // +10 dB at -64
        WifiRoam::Seen weakBetter[] = {seen(FAR, -66)};
        assert(check(r, NEAR, -86, weakBetter, 1, 0) == WifiRoam::STAY_TOO_WEAK); // +20 dB but -66
        WifiRoam::Seen edge[] = {seen(FAR, -65)};
        assert(check(r, NEAR, -75, edge, 1, 0) == WifiRoam::MOVE);               // exactly at both limits

        // Our own node listed stronger than WiFi.RSSI() says: never a "move" to ourselves,
        // and the scan's reading of our node is the one compared.
        WifiRoam::Seen self[] = {seen(NEAR, -30), seen(FAR, -45)};
        assert(check(r, NEAR, -60, self, 2, 0, &best) == WifiRoam::STAY_NOT_BETTER && best == 1);
        WifiRoam::Seen selfWeak[] = {seen(FAR, -50), seen(NEAR, -60)};
        assert(check(r, NEAR, -40, selfWeak, 2, 0, &best) == WifiRoam::MOVE && best == 0);

        // The strongest of several other nodes is the candidate.
        WifiRoam::Seen many[] = {seen(FARTHEST, -61), seen(FAR, -55), seen(NEAR, -70)};
        assert(check(r, NEAR, -70, many, 3, 0, &best) == WifiRoam::MOVE && best == 1);
    }

    // ---- cases seen on real boards (a mesh with one node next to the radar) ----
    {
        WifiRoam r;
        r.onConnected(FAR, 0);
        int best;
        // After a mesh restart, two boards sat on a far node (-55 / -60) with the near one
        // at -21 / -31.
        WifiRoam::Seen c1175[] = {seen(FAR, -55), seen(NEAR, -21)};
        assert(check(r, FAR, -55, c1175, 2, 3 * MIN, &best) == WifiRoam::MOVE && best == 1);
        WifiRoam::Seen c121[] = {seen(FAR, -60), seen(NEAR, -31)};
        assert(check(r, FAR, -60, c121, 2, 3 * MIN) == WifiRoam::MOVE);
        // After a mesh firmware update, both rejoined the farthest node (about -61).
        WifiRoam::Seen c2[] = {seen(FARTHEST, -61), seen(NEAR, -33)};
        assert(check(r, FARTHEST, -61, c2, 2, 3 * MIN) == WifiRoam::MOVE);
        // On the near node at -39 with a far one heard at about -55: stays.
        WifiRoam::Seen settled[] = {seen(NEAR, -39), seen(FAR, -55)};
        assert(check(r, NEAR, -39, settled, 2, 3 * MIN) == WifiRoam::STAY_NOT_BETTER);
    }

    // ---- moves: the gap after a move, and the back-off when it rejoins the same node ----
    {
        WifiRoam r;
        uint32_t t = 0;
        r.onConnected(FAR, t);
        t += 3 * MIN;
        assert(r.due(t) == WifiRoam::DUE_PERIODIC);
        r.onMoveStarted(FAR, t);
        assert(r.due(t + 1) == WifiRoam::NOT_DUE);                  // nothing while moving
        assert(r.due(t + 60 * MIN) == WifiRoam::NOT_DUE);           // ...however long it takes
        t += 3000;                                                  // a 3 s drop
        assert(r.onConnected(NEAR, t) == WifiRoam::MOVED);
        assert(r.periodMs() == WIFI_ROAM_PERIOD_MS);
        assert(r.due(t + 10 * MIN - 1) == WifiRoam::NOT_DUE);       // at most one move per 10 min
        for (int i = 0; i < 3; i++) r.onPollRssi(-80);              // even on a weak link
        assert(r.due(t + 9 * MIN) == WifiRoam::NOT_DUE);
        assert(r.due(t + 10 * MIN) == WifiRoam::DUE_PERIODIC);

        // The driver rejoins the node we tried to leave: the period doubles, up to 40 min.
        r.onMoveStarted(NEAR, t + 10 * MIN);
        t += 10 * MIN + 3000;
        assert(r.onConnected(NEAR, t) == WifiRoam::SAME_NODE);
        assert(r.periodMs() == 20 * MIN);
        assert(r.due(t + 20 * MIN - 1) == WifiRoam::NOT_DUE);
        assert(r.due(t + 20 * MIN) == WifiRoam::DUE_PERIODIC);
        r.onMoveStarted(NEAR, t + 20 * MIN);
        t += 20 * MIN + 3000;
        assert(r.onConnected(NEAR, t) == WifiRoam::SAME_NODE && r.periodMs() == 40 * MIN);
        r.onMoveStarted(NEAR, t + 40 * MIN);
        t += 40 * MIN + 3000;
        assert(r.onConnected(NEAR, t) == WifiRoam::SAME_NODE && r.periodMs() == WIFI_ROAM_MAX_PERIOD_MS);
        // A real move resets it.
        r.onMoveStarted(NEAR, t + 40 * MIN);
        t += 40 * MIN + 3000;
        assert(r.onConnected(FAR, t) == WifiRoam::MOVED && r.periodMs() == WIFI_ROAM_PERIOD_MS);

        // A plain reconnect (not ours) is a fresh join: 3 min, not a move result.
        r.onDisconnected();
        t += 60 * MIN;
        assert(r.onConnected(NEAR, t) == WifiRoam::JOINED);
        assert(r.due(t + 3 * MIN) == WifiRoam::DUE_PERIODIC);
    }

    // ---- the weak-signal trigger ----
    {
        WifiRoam r;
        r.onConnected(NEAR, 0);
        float avg = 0;
        assert(!r.averageRssi(&avg));                               // no samples yet
        r.onPollRssi(-75); r.onPollRssi(-75);
        assert(!r.weak());                                          // fewer than 3 samples
        assert(!r.averageRssi(&avg));
        r.onPollRssi(-75);
        assert(r.weak());
        assert(r.averageRssi(&avg) && avg == -75.0f);
        assert(r.due(2 * MIN - 1) == WifiRoam::NOT_DUE);            // 2 min gap after the join
        assert(r.due(2 * MIN) == WifiRoam::DUE_WEAK);
        WifiRoam::Seen s[] = {seen(NEAR, -75), seen(FAR, -84)};    // another node heard, too weak
        assert(r.decide(NEAR, -75, s, 2, 2 * MIN, nullptr, nullptr) == WifiRoam::STAY_TOO_WEAK);
        assert(r.due(4 * MIN - 1) == WifiRoam::NOT_DUE);            // ...and after each check
        assert(r.due(4 * MIN) == WifiRoam::DUE_WEAK);
        r.onPollRssi(-60); r.onPollRssi(-60); r.onPollRssi(-60);   // -60 avg: not weak
        assert(!r.weak() && r.due(4 * MIN) == WifiRoam::NOT_DUE);
        r.onPollRssi(-70); r.onPollRssi(-70); r.onPollRssi(-71);   // average -70.3: weak
        assert(r.weak());
        assert(r.averageRssi(&avg) && avg < -70.3f && avg > -70.4f);
        r.onPollRssi(-70); r.onPollRssi(-70); r.onPollRssi(-70);   // exactly -70: not weak
        assert(!r.weak());
        assert(r.averageRssi(&avg) && avg == -70.0f);
        r.onPollRssi(-60); r.onPollRssi(-76); r.onPollRssi(-76);   // only the last 3 count
        assert(r.averageRssi(&avg) && avg < -70.6f && avg > -70.7f && r.weak());
        r.onDisconnected();
        r.onConnected(NEAR, 5 * MIN);
        assert(!r.weak());                                          // samples restart on a join
        assert(!r.averageRssi(&avg));
    }
    {
        // A join long after the last check still waits the 2 min gap before a weak check.
        WifiRoam r;
        r.onConnected(NEAR, 0);
        r.onDisconnected();
        r.onConnected(NEAR, 50 * MIN);
        r.onPollRssi(-80); r.onPollRssi(-80); r.onPollRssi(-80);
        assert(r.due(50 * MIN + MIN) == WifiRoam::NOT_DUE);
        assert(r.due(52 * MIN) == WifiRoam::DUE_WEAK);
    }

    // ---- the all-channel fallback scan: weak link, nothing else heard, at most hourly ----
    {
        WifiRoam r;
        r.onConnected(NEAR, 0);
        WifiRoam::Seen alone[] = {seen(NEAR, -75)};
        WifiRoam::Seen other[] = {seen(NEAR, -75), seen(FAR, -80)};
        assert(!r.wantWideScan(alone, 1, NEAR, 3 * MIN));          // healthy link: no
        for (int i = 0; i < 3; i++) r.onPollRssi(-75);
        assert(!r.wantWideScan(other, 2, NEAR, 3 * MIN));          // another node was heard
        assert(r.wantWideScan(alone, 1, NEAR, 3 * MIN));           // weak and nothing else heard
        assert(!r.wantWideScan(nullptr, 0, NEAR, 3 * MIN + 59 * MIN));  // not again within the hour
        assert(r.wantWideScan(nullptr, 0, NEAR, 3 * MIN + 60 * MIN));
    }

    // ---- one access point: keep looking, but slow down ----
    {
        WifiRoam r;
        WifiRoam::Seen alone[] = {seen(NEAR, -40)};
        uint32_t t = 0;
        r.onConnected(NEAR, t);
        // Checks at 3, 13, 23 min hear nobody else; after the third the wait doubles.
        const uint32_t expectNext[] = {10, 10, 20, 40, 40, 40};
        t = 3 * MIN;
        for (int i = 0; i < 6; i++) {
            assert(r.due(t) == WifiRoam::DUE_PERIODIC);
            assert(r.decide(NEAR, -40, alone, 1, t, nullptr, nullptr) == WifiRoam::STAY_NO_OTHER);
            assert(r.aloneChecks() == i + 1);
            assert(r.due(t + expectNext[i] * MIN - 1) == WifiRoam::NOT_DUE);
            t += expectNext[i] * MIN;
        }
        // A node appears (added, or back after a restart): it is found within 40 min, and the
        // schedule is back to every 10 min.
        WifiRoam::Seen added[] = {seen(NEAR, -40), seen(FAR, -60)};
        assert(r.due(t) == WifiRoam::DUE_PERIODIC);
        assert(r.decide(NEAR, -40, added, 2, t, nullptr, nullptr) == WifiRoam::STAY_NOT_BETTER);
        assert(r.aloneChecks() == 0);
        assert(r.due(t + 10 * MIN - 1) == WifiRoam::NOT_DUE);
        assert(r.due(t + 10 * MIN) == WifiRoam::DUE_PERIODIC);

        // A rejoin starts over: first check at 3 min, then 10 min, even after a long quiet run.
        for (int i = 0; i < 5; i++) { t += 40 * MIN; r.decide(NEAR, -40, alone, 1, t, nullptr, nullptr); }
        assert(r.aloneChecks() == 5);
        r.onDisconnected();
        r.onConnected(NEAR, t + MIN);
        assert(r.aloneChecks() == 0);
        assert(r.due(t + 4 * MIN) == WifiRoam::DUE_PERIODIC);
        r.decide(NEAR, -40, alone, 1, t + 4 * MIN, nullptr, nullptr);
        assert(r.due(t + 14 * MIN) == WifiRoam::DUE_PERIODIC);
    }
    {
        // The mesh-restart case: only the far node is up at the first two checks; the near one
        // is back by the third and the radar moves, before any slow-down.
        WifiRoam r;
        WifiRoam::Seen onlyFar[] = {seen(FAR, -55)};
        WifiRoam::Seen back[] = {seen(FAR, -55), seen(NEAR, -21)};
        r.onConnected(FAR, 0);
        assert(r.decide(FAR, -55, onlyFar, 1, 3 * MIN, nullptr, nullptr) == WifiRoam::STAY_NO_OTHER);
        assert(r.decide(FAR, -55, onlyFar, 1, 13 * MIN, nullptr, nullptr) == WifiRoam::STAY_NO_OTHER);
        assert(r.due(23 * MIN) == WifiRoam::DUE_PERIODIC);
        assert(r.decide(FAR, -55, back, 2, 23 * MIN, nullptr, nullptr) == WifiRoam::MOVE);
    }
    {
        // A weak single access point (a lone router at -75, a phone hotspot): one weak-signal
        // check, then only the normal schedule, not a scan every 2 min.
        WifiRoam r;
        WifiRoam::Seen alone[] = {seen(NEAR, -75)};
        r.onConnected(NEAR, 0);
        for (int i = 0; i < 3; i++) r.onPollRssi(-75);
        assert(r.due(2 * MIN) == WifiRoam::DUE_WEAK);
        assert(r.decide(NEAR, -75, alone, 1, 2 * MIN, nullptr, nullptr) == WifiRoam::STAY_NO_OTHER);
        int checks = 1;
        for (uint32_t t = 2 * MIN + 1000; t < 3 * 60 * MIN; t += 1000) {   // the next 3 hours, 1 s steps
            const WifiRoam::Due d = r.due(t);
            assert(d != WifiRoam::DUE_WEAK);
            if (d == WifiRoam::DUE_PERIODIC) { r.decide(NEAR, -75, alone, 1, t, nullptr, nullptr); checks++; }
        }
        // 2 (weak), 12, 22, 42, 82, 122, 162 min: 7 checks in 3 h instead of about 90.
        assert(checks == 7);
        // Another node heard again: the weak-signal trigger is back.
        WifiRoam::Seen other[] = {seen(NEAR, -75), seen(FAR, -80)};
        r.decide(NEAR, -75, other, 2, 180 * MIN, nullptr, nullptr);
        assert(r.due(182 * MIN) == WifiRoam::DUE_WEAK);
    }
    {
        // The two slow-downs together: a same-node rejoin's 40 min isn't shortened by the
        // single-access-point schedule (which is still at 10 min there).
        WifiRoam r;
        WifiRoam::Seen alone[] = {seen(NEAR, -40)};
        uint32_t t = 0;
        r.onConnected(NEAR, t);
        for (int i = 0; i < 3; i++) {                // three moves that land on the same node
            r.onMoveStarted(NEAR, t);
            t += 3000;
            assert(r.onConnected(NEAR, t) == WifiRoam::SAME_NODE);
        }
        assert(r.periodMs() == WIFI_ROAM_MAX_PERIOD_MS);
        r.decide(NEAR, -40, alone, 1, t, nullptr, nullptr);
        assert(r.due(t + 40 * MIN - 1) == WifiRoam::NOT_DUE);
        assert(r.due(t + 40 * MIN) == WifiRoam::DUE_PERIODIC);
    }

    // ---- a move while the radar is in use waits; a failed scan doesn't retry at once ----
    {
        WifiRoam r;
        WifiRoam::Seen better[] = {seen(FAR, -55), seen(NEAR, -21)};
        r.onConnected(FAR, 0);
        assert(r.decide(FAR, -55, better, 2, 3 * MIN, nullptr, nullptr) == WifiRoam::MOVE);
        r.deferMove(3 * MIN);                                       // someone is tapping
        assert(r.due(3 * MIN + 30000 - 1) == WifiRoam::NOT_DUE);
        assert(r.due(3 * MIN + 30000) == WifiRoam::DUE_PERIODIC);   // fresh scan 30 s later
        assert(r.decide(FAR, -55, better, 2, 3 * MIN + 30000, nullptr, nullptr) == WifiRoam::MOVE);

        WifiRoam f;
        f.onConnected(NEAR, 0);
        assert(f.due(3 * MIN) == WifiRoam::DUE_PERIODIC);
        f.scanFailed(3 * MIN);
        assert(f.due(3 * MIN + 1000) == WifiRoam::NOT_DUE);
        assert(f.due(13 * MIN - 1) == WifiRoam::NOT_DUE);
        assert(f.due(13 * MIN) == WifiRoam::DUE_PERIODIC);
        assert(f.aloneChecks() == 0);                               // a failed scan isn't "alone"
    }

    printf("  wifi_roam: all checks passed\n");
    return 0;
}
