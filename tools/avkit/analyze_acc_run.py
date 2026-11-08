#!/usr/bin/env python3
"""analyze_acc_run.py - ACC metrics for one recorded session (with a lead vehicle).

  python3 tools/avkit/analyze_acc_run.py latest [--plots]
  python3 tools/avkit/analyze_acc_run.py <session dir or name> [--plots]

Reads the record_aeb_scenario.py streams odom_truth, lead_truth, acc_state,
binding, aeb_status and meta.json. Only the time the lead exists (its first to
last odometry row) is scored, so the ego standing at the start line before the
spawn never counts as following or as a standstill behind the lead.

True gap: Euclidean distance between the ego and the lead (both world-frame
ground-truth odometry) minus the ego's front half-length and the lead's rear
half-length - close to the along-lane gap at following distances, a little
short of it on a bend.

Writes acc_metrics.json (+ acc.png with --plots) into the session; AEB metrics
stay with analyze_aeb_scenario.py.
"""
import argparse
import json
import os
import sys

import numpy as np

EGO_FRONT = 2.25      # sedan.urdf.xacro ch_l / 2
LEAD_LENGTH = 6.0     # lead_vehicle.py LENGTH (box truck; sessions before it: --lead-length 4.0)
STANDSTILL_GAP = 5.0  # acc_params.yaml standstill_gap_m
TIME_GAP = 1.8        # acc_params.yaml time_gap_sec
STEADY_MIN_SEC = 5.0  # a following stretch counts as steady only this long
MOVING = 0.5          # m/s
STOPPED = 0.05        # m/s
LEAD_V_MAX = 20.0     # m/s, above any lead profile


def load(sess, name):
    p = os.path.join(sess, name + '.jsonl')
    if not os.path.exists(p):
        return []
    with open(p) as f:
        return [r for r in (json.loads(line) for line in f if line.strip()) if r.get('t', 1.0) > 0]


def r3(x):
    return None if x is None else round(float(x), 3)


def stats(e):
    return {'mean': r3(e.mean()), 'std': r3(e.std()), 'max_abs': r3(np.abs(e).max())}


def long_runs(mask, t, min_sec):
    """mask with every True run shorter than min_sec (in t) cleared."""
    out = np.zeros_like(mask)
    i, n = 0, len(mask)
    while i < n:
        if not mask[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and mask[j + 1]:
            j += 1
        if t[j] - t[i] >= min_sec:
            out[i:j + 1] = True
        i = j + 1
    return out


def resolve_session(arg, root):
    if arg == 'latest':
        names = sorted(d for d in os.listdir(root) if os.path.isdir(os.path.join(root, d)))
        if not names:
            sys.exit(f'no sessions in {root}')
        return os.path.join(root, names[-1])
    return arg if os.path.isdir(arg) else os.path.join(root, arg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('session')
    ap.add_argument('--sessions-root', default=os.path.expanduser('~/vlm_av_ws/data/aeb_scenarios'))
    ap.add_argument('--plots', action='store_true')
    ap.add_argument('--lead-length', type=float, default=LEAD_LENGTH,
                    help='lead vehicle length [m]; its odometry is at the centre, so half of it '
                         'is subtracted from the gap (default %(default)s)')
    a = ap.parse_args()
    sess = os.path.normpath(resolve_session(a.session, a.sessions_root))

    ego, lead = load(sess, 'odom_truth'), load(sess, 'lead_truth')
    # the OdometryPublisher differentiates poses, so the spawn teleport shows
    # up as one or two rows of thousands of m/s; no lead row is that fast
    lead = [r for r in lead if r['v'] < LEAD_V_MAX]
    acc, binding, status = load(sess, 'acc_state'), load(sess, 'binding'), load(sess, 'aeb_status')
    meta = {}
    if os.path.exists(os.path.join(sess, 'meta.json')):
        with open(os.path.join(sess, 'meta.json')) as f:
            meta = json.load(f)
    if not ego or not lead:
        sys.exit(f'{sess}: needs odom_truth and lead_truth rows')
    set_speed = meta.get('target_speed_mps')

    t_all = np.array([r['t'] for r in ego])
    v_all = np.array([r['v'] for r in ego])
    tl = np.array([r['t'] for r in lead])
    # the lead's lifetime only: np.interp would hold its first pose and speed
    # constant before the spawn
    w = (t_all >= tl[0]) & (t_all <= tl[-1])
    if w.sum() < 2:
        sys.exit(f'{sess}: ego and lead odometry do not overlap')
    t, v_ego = t_all[w], v_all[w]
    ex = np.array([r['x'] for r in ego])[w]
    ey = np.array([r['y'] for r in ego])[w]
    lx = np.interp(t, tl, [r['x'] for r in lead])
    ly = np.interp(t, tl, [r['y'] for r in lead])
    v_lead = np.interp(t, tl, [r['v'] for r in lead])
    gap = np.hypot(lx - ex, ly - ey) - EGO_FRONT - a.lead_length / 2.0
    desired = STANDSTILL_GAP + TIME_GAP * v_ego

    m = {'session': os.path.basename(sess), 'set_speed_mps': set_speed,
         'lead_first_t': r3(tl[0]), 'scored_sec': r3(t[-1] - t[0])}
    m['min_gap_m'] = r3(gap.min())
    m['contact'] = bool((gap <= 0.05).any())
    m['gap_at_lead_appearance_m'] = r3(gap[0])
    m['peak_ego_speed_mps'] = r3(v_all.max())
    if set_speed is not None:
        m['max_over_set_speed_mps'] = r3(max(0.0, v_all.max() - set_speed))

    # steady following: both moving, speeds matched, for STEADY_MIN_SEC in a row
    steady = long_runs((v_ego > MOVING) & (np.abs(v_ego - v_lead) < 0.2), t, STEADY_MIN_SEC)
    m['steady_sec'] = r3(np.sum(np.diff(t, prepend=t[0])[steady]))
    if steady.any():
        m['steady_gap_error_m'] = stats(gap[steady] - desired[steady])
        m['steady_time_gap_s'] = r3(np.median((gap[steady] - STANDSTILL_GAP) / v_ego[steady]))
    # a joint standstill counts only once the ego has driven: at the start of
    # the lead window the ego is still parked and the lead not yet moving
    driven = np.cumsum(v_ego > MOVING) > 0
    stopped = driven & (v_ego < STOPPED) & (v_lead < STOPPED)
    if stopped.any():
        m['standstill_gap_m'] = r3(np.median(gap[stopped]))
        # restart: the lead moves off after a joint standstill; how long until
        # the ego is back within 0.2 m/s of it (or at least at MOVING speed)
        i_stop = int(np.argmax(stopped))
        go = np.nonzero((np.arange(len(t)) > i_stop) & (v_lead > MOVING))[0]
        if go.size:
            i_go = go[0]
            back = np.nonzero((np.arange(len(t)) >= i_go) & (v_ego > MOVING)
                              & (v_ego >= v_lead - 0.2))[0]
            m['restart_lag_s'] = r3(t[back[0]] - t[i_go]) if back.size else None

    if acc:
        ta = np.array([r['t'] for r in acc])
        has = np.array([r['has_lead'] for r in acc])
        est = np.array([r['gap'] for r in acc])
        cap = np.array([r['cap'] for r in acc])
        inside = (ta >= t[0]) & (ta <= t[-1])
        sel = has & inside
        if sel.any():
            e = est[sel] - np.interp(ta[sel], t, gap)
            m['acc_gap_error_m'] = stats(e)
            m['acc_lead_speed_error_mps'] = r3(np.mean(np.abs(
                np.array([r['v_lead'] for r in acc])[sel] - np.interp(ta[sel], t, v_lead))))
        m['acc_has_lead_fraction'] = r3(has[inside].mean()) if inside.any() else None
        # false leads: ACC tracking something while no lead vehicle exists
        m['acc_has_lead_before_spawn_ticks'] = int((has & (ta < t[0])).sum())
        # reaction: first tick after the lead appears whose cap is below the set speed
        if set_speed is not None:
            dropped = np.nonzero(sel & (cap < set_speed))[0]
            m['cap_drop_latency_s'] = r3(ta[dropped[0]] - t[0]) if dropped.size else None

    if binding:
        kinds = {}
        for r in binding:
            k = r['binding'].split(':')[0]
            kinds[k] = kinds.get(k, 0) + 1
        m['binding_counts'] = kinds
        first_ok = next((r['t'] for r in binding if not r['binding'].startswith('hold')), None)
        m['hold_after_start_ticks'] = 0 if first_ok is None else sum(
            1 for r in binding if r['t'] > first_ok and r['binding'].startswith('hold'))

    states = [r['state'] for r in status]
    m['aeb_braking_ticks'] = states.count('BRAKING')
    m['aeb_intervened'] = 'BRAKING' in states
    near = [r for r in status if abs(r['t'] - t[0]) <= 0.5]
    if near:
        m['aeb_d_safe_at_appearance_m'] = r3(min(r['d_safe_m'] for r in near))

    with open(os.path.join(sess, 'acc_metrics.json'), 'w') as f:
        json.dump(m, f, indent=1)
    print(json.dumps(m, indent=1))

    if a.plots:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        t0 = t_all[0]
        fig, ax = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
        ax[0].plot(t_all - t0, v_all, 'k', label='ego')
        ax[0].plot(tl - t0, [r['v'] for r in lead], 'b', label='lead')
        if acc:
            ax[0].plot(ta - t0, np.minimum(cap, 5.0), 'g--', label='ACC cap (clipped at 5)')
        if set_speed is not None:
            ax[0].axhline(set_speed, color='0.6', lw=0.8, ls=':', label='set speed')
        ax[0].set_ylabel('speed [m/s]')
        ax[0].legend(fontsize=8)
        ax[1].plot(t - t0, gap, 'k', label='true gap')
        ax[1].plot(t - t0, desired, 'g:', label='desired gap')
        if acc and sel.any():
            ax[1].plot(ta[sel] - t0, est[sel], 'b.', ms=2, label='ACC estimate')
        ax[1].set_ylabel('gap [m]')
        ax[1].set_xlabel('sim time since recording start [s]')
        ax[1].legend(fontsize=8)
        fig.suptitle(m['session'])
        fig.tight_layout()
        fig.savefig(os.path.join(sess, 'acc.png'), dpi=110)


if __name__ == '__main__':
    main()
