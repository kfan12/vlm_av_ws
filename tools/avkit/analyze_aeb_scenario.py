#!/usr/bin/env python3
"""analyze_aeb_scenario.py - metrics for one recorded AEB scenario.

  python3 tools/avkit/analyze_aeb_scenario.py latest [--plots]

No ROS, no pandas (numpy; matplotlib only behind --plots). Reads a session
dir from record_aeb_scenario.py and writes metrics.json + report.md (+ plots
.png) into it.

Ground truth: meta.json carries the map's road_block pose and any
spawn_aeb_obstacle.sh boxes (--obstacle); the true gap from the sedan's front
bumper to an obstacle's near face is

    gt_standoff = (obs - ego_xy) . obs_heading - obs_depth/2 - BUMPER_X

with ego_xy from /odom_truth (absolute world frame, same frame as the map).
BUMPER_X = ch_l/2 = 2.25 m (sedan.urdf.xacro; base_link is chassis centre).
With several obstacles the nearest one in the car's path counts; a spawned box
only counts between its spawn and remove times (wall clock, mapped to sim time
through clock.jsonl). This is what AEB's own obstacle_distance_m is measured
against.

Arbiter (aeb:=true): /control/active_source gives source_sequence, the
command-gate response (emergency_active -> source 'aeb') and, per non-'mpc'
source, how fast the car actually stopped. Shadow sessions (aeb:=false, or
recorded before the arbiter existed) have no active_source rows and skip it.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

BUMPER_X = 2.25           # sedan.urdf.xacro ch_l / 2
HALF_WIDTH = 0.9          # sedan.urdf.xacro ch_w / 2
EMERGENCY_DECEL = 6.0     # aeb_params.yaml ego_emergency_decel_mps2
RESPONSE_TIME = 0.3       # aeb_params.yaml response_time_sec
STOPPED_MPS = 0.05        # truth speed below this = standstill
STATE_ORDER = ['INACTIVE', 'MONITORING', 'WARNING', 'BRAKING', 'STOP_HOLD', 'FAULT']
# control_arbiter_node.cpp sources, nominal first
SOURCE_ORDER = ['mpc', 'aeb', 'fault_aeb_cmd_stale', 'fault_aeb_status',
                'stop_no_fresh_source', 'estop']
STREAMS = ('aeb_status', 'aeb_cmd', 'mpc_cmd', 'cmd_vel', 'active_source', 'odom_truth',
           'odom_ekf', 'lane_path', 'plan_path', 'target_speed', 'clock')


def load_jsonl(path):
    rows = []
    if os.path.exists(path):
        with open(path) as f:
            for line in f:
                line = line.strip()
                if line:
                    r = json.loads(line)
                    if r.get('t', 1.0) > 0.0:   # t=0: recorded before /clock arrived
                        rows.append(r)
    return rows


def r3(x):
    return None if x is None else round(float(x), 3)


def interp(t_query, t, y):
    if len(t) == 0:
        return None
    return float(np.interp(t_query, t, y))


def gaps(rows, t0=None, t1=None):
    t = np.array([r['t'] for r in rows if (t0 is None or r['t'] >= t0) and (t1 is None or r['t'] <= t1)])
    if len(t) < 2:
        return None, None
    d = np.diff(t)
    return float(np.median(d)), float(d.max())


def wall_to_sim(wall, clock, before, after):
    """Sim time for a wall-clock instant; `before`/`after` if outside the recording."""
    if wall is None:
        return after
    if not clock or wall < clock[0]['wall']:
        return before
    if wall > clock[-1]['wall']:
        return after
    return float(np.interp(wall, [c['wall'] for c in clock], [c['t'] for c in clock]))


def obstacle_gt(ob, odom, to):
    """Per-sample bumper-to-face gap for one obstacle (NaN while out of path or absent),
    plus the lateral offset of the obstacle from the car."""
    hx, hy = math.cos(ob['yaw']), math.sin(ob['yaw'])
    gt = np.array([(ob['x'] - r['x']) * hx + (ob['y'] - r['y']) * hy
                   - ob['size'][0] / 2.0 - BUMPER_X for r in odom])
    # standoff only means something while the obstacle overlaps the car's
    # lateral extent; elsewhere (e.g. in the oncoming lane) the car can never
    # reach it, so the along-road gap is masked to NaN.
    lat = np.array([-(ob['x'] - r['x']) * hy + (ob['y'] - r['y']) * hx for r in odom])
    in_path = np.abs(lat) < ob['size'][1] / 2.0 + HALF_WIDTH
    present = (to >= ob['t_spawn']) & (to <= ob['t_remove'])
    return np.where(in_path & present, gt, np.nan), lat, in_path & present


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('session', help='session dir or "latest"')
    ap.add_argument('--sessions-root', default=os.path.expanduser('~/vlm_av_ws/data/aeb_scenarios'))
    ap.add_argument('--plots', action='store_true')
    args = ap.parse_args()

    sess = args.session
    if sess == 'latest':
        dirs = sorted(d for d in os.listdir(args.sessions_root)
                      if os.path.isdir(os.path.join(args.sessions_root, d)))
        if not dirs:
            sys.exit('no scenarios recorded yet')
        sess = os.path.join(args.sessions_root, dirs[-1])
    elif not os.path.isdir(sess):
        sess = os.path.join(args.sessions_root, sess)
    sess = os.path.normpath(sess)   # a trailing '/' would make basename() empty

    L ={n: load_jsonl(os.path.join(sess, n + '.jsonl')) for n in STREAMS}
    meta = {}
    if os.path.exists(os.path.join(sess, 'meta.json')):
        with open(os.path.join(sess, 'meta.json')) as f:
            meta = json.load(f)
    status = L['aeb_status']
    if not status:
        sys.exit('no aeb_status rows recorded')
    odom = L['odom_truth']

    m = {'session': os.path.basename(sess), 'world': meta.get('world'),
         'target_speed_mps': meta.get('target_speed_mps')}

    # ---------------- ground-truth standoff to the obstacles -----------------
    to = np.array([r['t'] for r in odom]) if odom else np.array([])
    vo = np.array([r['v'] for r in odom]) if odom else np.array([])
    gt = None
    rb = meta.get('road_block')
    spawned = meta.get('obstacles') or []
    if spawned and not L['clock']:
        print('warn: obstacles recorded but no clock.jsonl - treating them as present all run')
    targets = []
    if rb:
        targets.append(dict(rb, name='road_block', t_spawn=-math.inf, t_remove=math.inf))
    for ob in spawned:
        targets.append(dict(ob, t_spawn=wall_to_sim(ob.get('spawned_wall'), L['clock'], -math.inf, math.inf),
                            t_remove=wall_to_sim(ob.get('removed_wall'), L['clock'], -math.inf, math.inf)))
    if targets and odom:
        per = []
        for ob in targets:
            g, lat, reach = obstacle_gt(ob, odom, to)
            per.append(g)
            if ob['name'] == 'road_block':
                m['road_block'] = rb
                m['block_in_ego_path'] = bool(reach.any())
                m['min_lateral_offset_to_block_m'] = r3(np.abs(lat).min())
            else:
                m.setdefault('obstacles', []).append({
                    'name': ob['name'], 'x': ob['x'], 'y': ob['y'], 's': ob.get('s'),
                    'lateral': ob.get('lateral'), 'size': ob['size'],
                    't_spawn': r3(ob['t_spawn']) if math.isfinite(ob['t_spawn']) else None,
                    't_remove': r3(ob['t_remove']) if math.isfinite(ob['t_remove']) else None,
                    'in_ego_path': bool(reach.any()),
                    'min_lateral_offset_m': r3(np.abs(lat[(to >= ob['t_spawn']) & (to <= ob['t_remove'])]).min())
                    if ((to >= ob['t_spawn']) & (to <= ob['t_remove'])).any() else None})
        gt = np.fmin.reduce(np.vstack(per), axis=0)   # nearest reachable obstacle, NaN if none
        if np.isnan(gt).all():
            gt = None
    if gt is not None:
        m['min_gt_standoff_m'] = r3(np.nanmin(gt))
        # contact: bumper reaches the block face (2 cm tolerance), else the
        # car stalls within 0.3 m of it (collision geometry stops it).
        contact = np.where(np.nan_to_num(gt, nan=1e9) <= 0.02)[0]
        if len(contact) == 0:
            contact = np.where((np.nan_to_num(gt, nan=1e9) <= 0.3) & (vo < STOPPED_MPS))[0]
        m['contact'] = bool(len(contact))
        if len(contact):
            i = int(contact[0])
            m['contact_t'] = r3(to[i])
            m['impact_speed_mps'] = r3(vo[max(0, i - 5):i + 1].max())

    # ---------------- AEB state timeline -------------------------------------
    ts = np.array([r['t'] for r in status])
    states = [r['state'] for r in status]
    timeline = []
    for i, s in enumerate(states):
        if i == 0 or s != states[i - 1]:
            timeline.append({'t': r3(ts[i]), 'state': s})
    m['state_timeline'] = timeline
    m['state_counts'] = {s: states.count(s) for s in STATE_ORDER if states.count(s)}
    med, mx = gaps(status)
    m['aeb_status_rate_hz'] = r3(1.0 / med) if med else None
    m['aeb_status_max_gap_s'] = r3(mx)
    m['fault_reasons'] = sorted({r['reason'] for r in status if r['state'] == 'FAULT'})

    def at(t):
        out = {'t': r3(t)}
        if len(to):
            out['truth_speed_mps'] = r3(interp(t, to, vo))
        if gt is not None:
            g = interp(t, to, gt)
            out['gt_standoff_m'] = None if g is None or math.isnan(g) else r3(g)
        return out

    def standstill_after(t):
        """First sim time >= t with truth speed below STOPPED_MPS, or None."""
        idx = np.where((to >= t) & (vo < STOPPED_MPS))[0] if len(to) else []
        return float(to[idx[0]]) if len(idx) else None

    for name in ('WARNING', 'BRAKING', 'STOP_HOLD'):
        idx = next((i for i, s in enumerate(states) if s == name), None)
        if idx is None:
            continue
        r = status[idx]
        d = at(r['t'])
        d.update({'perceived_distance_m': r['obstacle_distance_m'], 'ttc_sec': r['ttc_sec'],
                  'd_safe_m': r['d_safe_m'], 'aeb_speed_mps': r['relative_speed_mps']})
        if name == 'BRAKING' and d.get('gt_standoff_m') is not None:
            v = d['truth_speed_mps']
            need = v * v / (2 * EMERGENCY_DECEL)
            d['stop_margin_m'] = r3(d['gt_standoff_m'] - need)
            d['stop_margin_with_response_m'] = r3(d['gt_standoff_m'] - need - v * RESPONSE_TIME)
            if m.get('contact_t') is not None:
                d['time_before_contact_s'] = r3(m['contact_t'] - r['t'])
        m['first_' + name.lower()] = d

    # first time anything was perceived at all (any in-corridor cluster)
    seen = [r for r in status if r['obstacle_distance_m'] >= 0]
    if seen:
        d = at(seen[0]['t'])
        d['perceived_distance_m'] = seen[0]['obstacle_distance_m']
        m['first_detection'] = d

    # ---------------- perception accuracy vs ground truth ---------------------
    if gt is not None:
        rows = [r for r in status if r['obstacle_distance_m'] >= 0]
        if rows:
            t_r = np.array([r['t'] for r in rows])
            gt_r = np.interp(t_r, to, gt)
            pd = np.array([r['obstacle_distance_m'] for r in rows])
            ok = gt_r > 0.05                 # before contact only (NaN compares False)
            err = pd[ok] - gt_r[ok]
            if len(err):
                m['distance_error_m'] = {'n': int(len(err)), 'mean': r3(err.mean()),
                                         'std': r3(err.std()), 'max_abs': r3(np.abs(err).max())}
                # fraction of ticks, while an obstacle was 0-30 m ahead, in which
                # AEB saw anything in the corridor at all
                win = (np.interp(ts, to, gt) > 0.05) & (np.interp(ts, to, gt) < 30)
                if win.any():
                    det = np.array([r['obstacle_distance_m'] >= 0 for r in status])[win]
                    m['detection_rate_0_30m'] = r3(det.mean())
            # ticks reporting an obstacle much nearer than the real one (phantom)
            m['phantom_ticks_gt_minus_3m'] = int(((gt_r - pd) > 3.0).sum())

    # ENTERING BRAKING with nothing in the corridor is a bug by construction;
    # staying in BRAKING after the cluster vanished is the designed latch
    # (release only via STOP_HOLD), so it is counted separately.
    risky = [r for r in status if r['state'] in ('WARNING', 'BRAKING')]
    brake_entries = [r for i, r in enumerate(status)
                     if r['state'] == 'BRAKING' and (i == 0 or status[i - 1]['state'] != 'BRAKING')]
    m['braking_entries_without_obstacle'] = sum(1 for r in brake_entries if r['obstacle_distance_m'] < 0)
    m['latched_braking_ticks_without_obstacle'] = sum(
        1 for r in risky if r['state'] == 'BRAKING' and r['obstacle_distance_m'] < 0)
    # a BRAKING entry with no real obstacle in the car's path
    # at that moment (none in the world, out of lane, not spawned yet / removed)
    # or with nothing in AEB's own corridor is a false positive.
    gt_at = (lambda t: interp(t, to, gt)) if gt is not None else (lambda t: math.nan)
    m['false_positive_brakes'] = sum(
        1 for r in brake_entries
        if r['obstacle_distance_m'] < 0 or gt_at(r['t']) is None or math.isnan(gt_at(r['t'])))
    if gt is None:
        # no reachable obstacle all run: any WARNING/BRAKING tick is spurious
        m['false_positive_warning_ticks'] = sum(1 for r in risky if r['state'] == 'WARNING')
        m['false_positive_braking_ticks'] = sum(1 for r in risky if r['state'] == 'BRAKING')
        m['false_positive_events'] = [{'t': r['t'], 'state': r['state'],
                                       'd': r['obstacle_distance_m'], 'ttc': r['ttc_sec']}
                                      for r in risky][:30]

    # ---------------- command authority (control_arbiter) ---------------------
    src = L['active_source']
    m['arbiter_present'] = bool(src)
    if src:
        names = [r['source'] for r in src]
        m['source_sequence'] = [s for i, s in enumerate(names) if i == 0 or s != names[i - 1]]
        m['source_timeline'] = [{'t': r['t'], 'source': r['source']} for i, r in enumerate(src)
                                if i == 0 or r['source'] != src[i - 1]['source']]
        m['source_counts'] = {s: names.count(s) for s in dict.fromkeys(SOURCE_ORDER + names)
                              if names.count(s)}
        med, mx = gaps(src)
        m['active_source_rate_hz'] = r3(1.0 / med) if med else None
        m['active_source_max_gap_s'] = r3(mx)

        # first entry into every non-nominal source: speed/standoff then, and
        # how long the car took to actually stand still
        events = {}
        for e in m['source_timeline']:
            if e['source'] == 'mpc' or e['source'] in events:
                continue
            d = at(e['t'])
            t_stop = standstill_after(e['t'])
            d['time_to_standstill_s'] = r3(t_stop - e['t']) if t_stop is not None else None
            if t_stop is not None and gt is not None:
                d['standstill_gt_standoff_m'] = at(t_stop).get('gt_standoff_m')
            events[e['source']] = d
        m['source_events'] = events

        # command-gate response: AebStatus.emergency_active first true -> the
        # arbiter's first 'aeb' tick (a few arbiter ticks expected).
        emer = next((r for r in status if r['emergency_active']), None)
        if emer is not None:
            m['first_emergency_t'] = emer['t']
            gate = next((r for r in src if r['t'] >= emer['t'] and r['source'] == 'aeb'), None)
            m['command_gate_response_sec'] = r3(gate['t'] - emer['t']) if gate else None

    # cmd-side measure: emergency_active first true ->
    # /cmd_vel first at or below half its value at that moment. Includes the
    # AEB command's own deceleration ramp, so it is longer than the gate time.
    emer = next((r for r in status if r['emergency_active']), None)
    cmd = L['cmd_vel']
    if emer is not None and cmd:
        v0 = next((c['v'] for c in cmd if c['t'] >= emer['t']), None)
        if v0:
            resp = next((c for c in cmd if c['t'] >= emer['t'] and c['v'] <= v0 / 2.0), None)
            if resp:
                m['cmd_vel_half_speed_response_sec'] = r3(resp['t'] - emer['t'])

    # ---------------- after contact / at standstill ---------------------------
    if m.get('contact_t') is not None:
        after = [r['state'] for r in status if r['t'] >= m['contact_t'] + 1.0]
        if after:
            m['states_after_contact'] = {s: after.count(s) for s in STATE_ORDER if after.count(s)}
            m['state_at_end'] = after[-1]

        # speed AEB believes (from /odom_ekf) vs truth, once the car is pinned
        # against the block: wheel slip makes wheel-odom/EKF speed meaningless
        if len(to):
            late = [r for r in status if r['t'] >= m['contact_t'] + 1.0]
            if late:
                m['after_contact_aeb_speed_mps_mean'] = r3(np.mean([r['relative_speed_mps'] for r in late]))
                m['after_contact_truth_speed_mps_mean'] = r3(np.mean(np.interp([r['t'] for r in late], to, vo)))

    # detection coverage by true range band (obstacle ahead, before contact)
    if gt is not None:
        gts = np.interp(ts, to, gt)
        det = np.array([r['obstacle_distance_m'] >= 0 for r in status])
        bands = {}
        for lo, hi in ((0.05, 5), (5, 10), (10, 15), (15, 20), (20, 30)):
            w = (gts > lo) & (gts <= hi)
            if w.any():
                bands[f'{lo:g}-{hi:g}m'] = r3(det[w].mean())
        m['detection_rate_by_range'] = bands

    # ---------------- upstream health (confound check) ------------------------
    t_end = m.get('contact_t')
    for name in ('lane_path', 'plan_path'):
        med, mx = gaps(L[name], t1=t_end)
        m[name + '_rate_hz'] = r3(1.0 / med) if med else None
        m[name + '_max_gap_s'] = r3(mx)
    if odom:
        m['peak_truth_speed_mps'] = r3(vo.max())
        m['final_truth_speed_mps'] = r3(vo[-1])
    if cmd and t_end is not None:
        pre = [c['v'] for c in cmd if c['t'] <= t_end]
        m['cmd_vel_at_contact_mps'] = r3(pre[-1]) if pre else None

    with open(os.path.join(sess, 'metrics.json'), 'w') as f:
        json.dump(m, f, indent=1)

    lines = ['# AEB scenario report - ' + m['session'], '']
    for k, v in m.items():
        lines.append(f'- **{k}**: {json.dumps(v)}')
    report = '\n'.join(lines) + '\n'
    with open(os.path.join(sess, 'report.md'), 'w') as f:
        f.write(report)
    print(report)

    if args.plots:
        plot(sess, m, L, status, to, vo, gt)


def plot(sess, m, L, status, to, vo, gt):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    src = L['active_source']
    ts = np.array([r['t'] for r in status])
    t0 = min([ts[0]] + ([to[0]] if len(to) else []))
    fig, ax = plt.subplots(5 if src else 4, 1, figsize=(10, 12 if src else 10), sharex=True)

    if len(to):
        ax[0].plot(to - t0, vo, label='truth speed', color='k')
    cmd = L['cmd_vel']
    if cmd:
        ax[0].plot([c['t'] - t0 for c in cmd], [c['v'] for c in cmd],
                   label='/cmd_vel' + (' (arbiter)' if src else ' (MPC)'), alpha=0.7)
    mc = L['mpc_cmd']
    if mc:
        ax[0].plot([c['t'] - t0 for c in mc], [c['v'] for c in mc], label='/control/mpc_cmd',
                   alpha=0.6, ls=':')
    ac = L['aeb_cmd']
    if ac:
        ax[0].plot([c['t'] - t0 for c in ac], [c['v'] for c in ac],
                   label='/control/aeb_cmd' + ('' if src else ' (shadow)'), alpha=0.7, ls='--')
    ax[0].set_ylabel('speed [m/s]')
    ax[0].legend(loc='upper right', fontsize=8)

    pd = np.array([r['obstacle_distance_m'] for r in status])
    pd_plot = np.where(pd >= 0, pd, np.nan)
    ax[1].plot(ts - t0, pd_plot, '.', ms=3, label='AEB obstacle_distance_m')
    ax[1].plot(ts - t0, [r['d_safe_m'] for r in status], label='d_safe', alpha=0.8)
    if gt is not None:
        ax[1].plot(to - t0, gt, color='k', label='ground-truth standoff')
    ax[1].set_ylabel('distance [m]')
    ax[1].set_ylim(-1, 35)
    ax[1].legend(loc='upper right', fontsize=8)

    ttc = np.array([r['ttc_sec'] for r in status])
    ax[2].plot(ts - t0, np.where(ttc < 900, ttc, np.nan), '.', ms=3, label='TTC')
    ax[2].axhline(2.5, ls=':', color='orange', label='warning_ttc')
    ax[2].axhline(1.5, ls=':', color='red', label='brake_ttc')
    ax[2].set_ylim(0, 12)
    ax[2].set_ylabel('TTC [s]')
    ax[2].legend(loc='upper right', fontsize=8)

    sidx = [STATE_ORDER.index(r['state']) if r['state'] in STATE_ORDER else -1 for r in status]
    ax[3].step(ts - t0, sidx, where='post')
    ax[3].set_yticks(range(len(STATE_ORDER)))
    ax[3].set_yticklabels(STATE_ORDER, fontsize=8)

    if src:
        order = SOURCE_ORDER + sorted({r['source'] for r in src} - set(SOURCE_ORDER))
        ax[4].step([r['t'] - t0 for r in src], [order.index(r['source']) for r in src], where='post')
        ax[4].set_yticks(range(len(order)))
        ax[4].set_yticklabels(order, fontsize=8)
        ax[4].set_ylabel('active_source')
    ax[-1].set_xlabel('sim time [s]')
    for ob in m.get('obstacles', []):
        for k, c in (('t_spawn', 'g'), ('t_remove', 'b')):
            if ob.get(k) is not None:
                for a in ax:
                    a.axvline(ob[k] - t0, color=c, alpha=0.3, ls='--')
    if m.get('contact_t') is not None:
        for a in ax:
            a.axvline(m['contact_t'] - t0, color='r', alpha=0.4)
    fig.suptitle(m['session'])
    fig.tight_layout()
    out = os.path.join(sess, 'timeline.png')
    fig.savefig(out, dpi=110)
    print('wrote', out)


if __name__ == '__main__':
    main()
