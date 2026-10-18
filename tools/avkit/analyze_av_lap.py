#!/usr/bin/env python3
"""analyze_av_lap.py - metrics + report for a recorded lap (Day 15).

  python3 tools/avkit/analyze_av_lap.py latest [--map maps/urban_course.json] [--plots]

No ROS, no pandas: numpy (+ matplotlib only for --plots). Reads a session dir
from record_av_lap.py and writes metrics.json + report.md (+ lap.png) into it.

Frames: everything is compared in the WORLD frame (the map JSON's frame).
/odom_truth is already absolute world; /odom_ekf is spawn-zeroed and is moved
into world by the map's spawn pose. (day15.md shifted the map into odom and
then compared it against raw /odom_truth, which double-counts the spawn pose.)

Reference lane: the map's course centerline offset half a lane RIGHT of the
travel direction (the same construction as local_planner's build_course_lane).
Ego track: /odom_truth when recorded, else /odom_ekf.

Completion: completion_pct is max arc length reached over the whole right
lane; completion_to_stop_pct is the same over the arc length of the map's stop
line, so a lap that correctly stops ~2.5 m short of the line reads ~99 % there
and ~96 % on completion_pct. stop_standoff_m is the along-lane gap from
base_link (chassis centre) to the stop line; stop_standoff_bumper_m subtracts
the half-length BUMPER_X.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

BUMPER_X = 2.25           # sedan.urdf.xacro ch_l / 2
MOVING_MPS = 0.1
STOPPED_MPS = 0.05
SETPOINT_MIN_HOLD_S = 0.5  # target_speed values held shorter than this are blips
END_CAP_M = 0.1            # samples projecting within this of a lane end are off the lane


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


def odom_to_world(xy, spawn):
    """Spawn-zeroed odom -> world (inverse of the stack's map_origin_* shift)."""
    c, s = math.cos(spawn['yaw']), math.sin(spawn['yaw'])
    return np.column_stack([spawn['x'] + c * xy[:, 0] - s * xy[:, 1],
                            spawn['y'] + s * xy[:, 0] + c * xy[:, 1]])


def right_lane(centerline, half):
    """Offset the two-way road centerline half a lane RIGHT of travel
    (right-hand traffic; sample order = direction of travel)."""
    lane = []
    n = len(centerline)
    for i in range(n):
        j = min(i + 1, n - 1)
        k = max(j - 1, 0)
        t = centerline[j] - centerline[k]
        nt = math.hypot(t[0], t[1])
        if nt < 1e-9:
            continue
        t = t / nt
        lane.append(centerline[i] + half * np.array([t[1], -t[0]]))
    return np.asarray(lane)


def project_all(poly, pts):
    """Project each point onto the polyline: arrays (s, dist), + total length."""
    a, b = poly[:-1], poly[1:]
    d = b - a
    L2 = np.maximum((d * d).sum(1), 1e-12)
    seg_len = np.sqrt(L2)
    s_cum = np.concatenate([[0.0], np.cumsum(seg_len)])
    out_s = np.empty(len(pts))
    out_dist = np.empty(len(pts))
    for i, q in enumerate(pts):
        t = np.clip(((q - a) * d).sum(1) / L2, 0.0, 1.0)
        foot = a + t[:, None] * d
        dist = np.hypot(*(q - foot).T)
        j = int(np.argmin(dist))
        out_s[i] = s_cum[j] + t[j] * seg_len[j]
        out_dist[i] = dist[j]
    return out_s, out_dist, float(s_cum[-1])


def setpoint_sequence(rows):
    """Run-length encode target_speed: [[v, t_start_rel, hold_s], ...], blips dropped."""
    if not rows:
        return []
    t0 = rows[0]['t']
    runs = []
    for r in rows:
        v = round(r['v'], 2)
        if runs and runs[-1][0] == v:
            runs[-1][2] = r['t']
        else:
            runs.append([v, r['t'], r['t']])
    out = []
    for v, ta, tb in runs:
        if tb - ta < SETPOINT_MIN_HOLD_S:
            continue
        if out and out[-1][0] == v:   # a dropped blip split one hold in two
            out[-1][2] = round(tb - t0 - out[-1][1], 1)
        else:
            out.append([v, round(ta - t0, 1), round(tb - ta, 1)])
    return out


def state_sequence(rows):
    seq = []
    for r in rows:
        if not seq or seq[-1] != r['state']:
            seq.append(r['state'])
    return seq


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('session', help='session dir or "latest"')
    ap.add_argument('--map', default=None,
                    help='map JSON (default: maps/<meta world>.json, else maps/urban_course.json)')
    ap.add_argument('--sessions-root', default=os.path.expanduser('~/vlm_av_ws/data/av_laps'))
    ap.add_argument('--plots', action='store_true')
    args = ap.parse_args()

    sess = args.session
    if sess == 'latest':
        dirs = sorted(d for d in os.listdir(args.sessions_root)
                      if os.path.isdir(os.path.join(args.sessions_root, d)))
        if not dirs:
            sys.exit('no sessions recorded yet')
        sess = os.path.join(args.sessions_root, dirs[-1])
    elif not os.path.isdir(sess):
        sess = os.path.join(args.sessions_root, sess)

    meta = {}
    if os.path.exists(os.path.join(sess, 'meta.json')):
        with open(os.path.join(sess, 'meta.json')) as f:
            meta = json.load(f)
    map_path = args.map or os.path.expanduser(
        f"~/vlm_av_ws/maps/{meta.get('world', 'urban_course')}.json")
    with open(map_path) as f:
        mapj = json.load(f)
    spawn = mapj['spawn']
    lane = right_lane(np.asarray(mapj['course']['centerline'], dtype=float),
                      mapj['lane_width'] / 2.0)

    truth = load_jsonl(os.path.join(sess, 'odom_truth.jsonl'))
    ekf = load_jsonl(os.path.join(sess, 'odom_ekf.jsonl'))
    if len(truth) >= 10:
        track_name, rows = 'odom_truth', truth
        xy = np.array([[r['x'], r['y']] for r in rows])
    elif len(ekf) >= 10:
        track_name, rows = 'odom_ekf', ekf
        xy = odom_to_world(np.array([[r['x'], r['y']] for r in rows]), spawn)
    else:
        sys.exit('no odometry recorded')
    t = np.array([r['t'] for r in rows])
    v = np.array([r['v'] for r in rows])

    s, lat, lane_len = project_all(lane, xy)
    # a sample past either end of the lane polyline projects onto the end cap,
    # where |distance| is longitudinal overshoot, not cross-track error
    on_lane = (s > END_CAP_M) & (s < lane_len - END_CAP_M)
    in_motion = v > MOVING_MPS
    moving = in_motion & on_lane

    m = {'session': os.path.basename(sess), 'world': meta.get('world'),
         'seed': mapj.get('seed'), 'track_source': track_name,
         'duration_s': round(float(t[-1] - t[0]), 1),
         'distance_m': round(float(np.hypot(*np.diff(xy, axis=0).T).sum()), 2),
         'lane_length_m': round(lane_len, 2)}
    m['completion_pct'] = round(100.0 * float(s.max()) / lane_len, 1)
    if moving.any():
        m['crosstrack_rmse_m'] = round(float(np.sqrt((lat[moving] ** 2).mean())), 4)
        m['crosstrack_max_m'] = round(float(lat[moving].max()), 4)
        m['crosstrack_max_at_s_m'] = round(float(s[moving][np.argmax(lat[moving])]), 1)
        t_move = t[in_motion]
        m['time_to_first_move_s'] = round(float(t_move[0] - t[0]), 1)
    # straight-line window: the first leg (s in 5..55 of the 60 m straight)
    win = moving & (s > 5.0) & (s < 55.0)
    if win.sum() > 20:
        m['straight_lat_std_m'] = round(float(lat[win].std()), 4)

    vref = load_jsonl(os.path.join(sess, 'v_ref.jsonl'))
    if len(vref) > 10 and in_motion.any():
        tr = np.array([r['t'] for r in vref])
        vr = np.array([r['v'] for r in vref])
        # first to last moving sample only: once mpc_tracker_v2 is in its
        # arrived-at-stop hold it stops refreshing /mpc/v_ref_ramped, which then
        # sits at a stale nonzero value for the rest of the recording
        sel = ((t >= max(tr[0], t[in_motion][0])) & (t <= min(tr[-1], t[in_motion][-1])))
        if sel.any():
            vi = np.interp(t[sel], tr, vr)
            m['speed_rmse_ms'] = round(float(np.sqrt(((v[sel] - vi) ** 2).mean())), 4)
    m['speed_max_ms'] = round(float(v.max()), 3)

    if 'stop_line' in mapj:
        sl = np.array([[mapj['stop_line']['x'], mapj['stop_line']['y']]])
        s_stop = float(project_all(lane, sl)[0][0])
        m['stop_line_s_m'] = round(s_stop, 2)
        m['completion_to_stop_pct'] = round(100.0 * float(s.max()) / s_stop, 1)
        tail = t >= t[-1] - 2.0
        m['stopped'] = bool((v[tail] < STOPPED_MPS).all())
        m['stop_standoff_m'] = round(s_stop - float(s[-1]), 3)
        m['stop_standoff_bumper_m'] = round(s_stop - float(s[-1]) - BUMPER_X, 3)

    if track_name == 'odom_truth' and len(ekf) >= 10:
        te = np.array([r['t'] for r in ekf])
        xe = odom_to_world(np.array([[r['x'], r['y']] for r in ekf]), spawn)
        sel = (t >= te[0]) & (t <= te[-1])
        if sel.any():
            dx = np.interp(t[sel], te, xe[:, 0]) - xy[sel, 0]
            dy = np.interp(t[sel], te, xe[:, 1]) - xy[sel, 1]
            drift = np.hypot(dx, dy)
            m['ekf_drift_max_m'] = round(float(drift.max()), 3)
            m['ekf_drift_final_m'] = round(float(drift[-1]), 3)

    target = load_jsonl(os.path.join(sess, 'target_speed.jsonl'))
    m['target_speed_sequence'] = setpoint_sequence(target)
    m['maneuver_state_sequence'] = state_sequence(load_jsonl(os.path.join(sess, 'state.jsonl')))

    aeb = load_jsonl(os.path.join(sess, 'aeb_state.jsonl'))
    if aeb:
        counts = {}
        for r in aeb:
            counts[r['state']] = counts.get(r['state'], 0) + 1
        m['aeb_state_counts'] = counts
        m['aeb_fault_episodes'] = sum(
            1 for a, b in zip([None] + aeb[:-1], aeb)
            if b['state'] == 'FAULT' and (a is None or a['state'] != 'FAULT'))
        # BRAKING/STOP_HOLD on an obstacle-free lap are false positives once an
        # arbiter acts on them: locate each episode on the lane
        episodes = []
        for r in aeb:
            active = r['state'] in ('BRAKING', 'STOP_HOLD')
            if active and (not episodes or episodes[-1]['_open'] is False):
                episodes.append({'_open': True, 't_start': r['t'], 't_end': r['t'],
                                 'states': [r['state']], 'min_dist': r.get('obstacle_distance_m'),
                                 'reasons': [r.get('reason', '')]})
            elif active:
                e = episodes[-1]
                e['t_end'] = r['t']
                if r['state'] not in e['states']:
                    e['states'].append(r['state'])
                d = r.get('obstacle_distance_m')
                if d is not None and (e['min_dist'] is None or d < e['min_dist']):
                    e['min_dist'] = d
                if r.get('reason', '') not in e['reasons']:
                    e['reasons'].append(r.get('reason', ''))
            elif episodes:
                episodes[-1]['_open'] = False
        m['aeb_brake_episodes'] = [
            {'t_rel_s': round(e['t_start'] - t[0], 1),
             'duration_s': round(e['t_end'] - e['t_start'], 2),
             's_m': round(float(np.interp(e['t_start'], t, s)), 1),
             'lat_m': round(float(np.interp(e['t_start'], t, lat)), 2),
             'states': e['states'],
             'min_obstacle_distance_m': e['min_dist'],
             'reasons': e['reasons'][:3]}
            for e in episodes]

    with open(os.path.join(sess, 'metrics.json'), 'w') as f:
        json.dump(m, f, indent=1)

    lines = ['# Lap report - ' + m['session'], '']
    for k, val in m.items():
        lines.append(f'- **{k}**: {val}')
    report = '\n'.join(lines) + '\n'
    with open(os.path.join(sess, 'report.md'), 'w') as f:
        f.write(report)
    print(report)

    if args.plots:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 3, figsize=(16, 5))
        ax[0].plot(lane[:, 0], lane[:, 1], 'k--', lw=1, label='map lane')
        ax[0].plot(xy[:, 0], xy[:, 1], 'b-', lw=1, label=track_name)
        if 'stop_line' in mapj:
            ax[0].plot(mapj['stop_line']['x'], mapj['stop_line']['y'], 'rx', label='stop line')
        ax[0].set_aspect('equal')
        ax[0].legend()
        ax[0].set_title('trajectory (world)')
        ax[1].plot(s[moving], lat[moving], '.', ms=2)
        ax[1].set_xlabel('s [m]')
        ax[1].set_ylabel('|cross-track| [m]')
        ax[1].set_title('cross-track vs arc length')
        ax[2].plot(t - t[0], v, label='v')
        if len(vref) > 10:
            ax[2].plot(tr - t[0], vr, label='v_ref')
        if target:
            ax[2].step([r['t'] - t[0] for r in target], [min(r['v'], 4.0) for r in target],
                       where='post', lw=0.8, label='target_speed (clipped at 4)')
        ax[2].set_xlabel('t [s]')
        ax[2].legend()
        ax[2].set_title('speed tracking')
        fig.tight_layout()
        fig.savefig(os.path.join(sess, 'lap.png'), dpi=120)
        print('wrote', os.path.join(sess, 'lap.png'))


if __name__ == '__main__':
    main()
