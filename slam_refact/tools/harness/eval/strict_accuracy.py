#!/usr/bin/env python3
"""P0 XR pose-log metrics with explicit provenance; never a product acceptance gate.

The legacy accuracy.py and its registered results remain unchanged. See
STRICT_ACCURACY.md for the required sidecar protocol and coordinate convention.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from accuracy import umeyama  # noqa: E402
from common import quat_wxyz_to_matrix, rotation_angle_deg, summary  # noqa: E402


class DiagnosticOnly(ValueError):
    """A recognized observation log that cannot supply metric evaluation input."""
    def __init__(self, message, diagnostics):
        super().__init__(message)
        self.diagnostics = diagnostics


def finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    return float(value)


def integer(value, name):
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f'{name} must be a nonnegative integer')
    return value


def vector(value, size, name):
    if not isinstance(value, list) or len(value) != size:
        raise ValueError(f'{name} must have {size} entries')
    return np.array([finite(x, name) for x in value])


def finite_array(value, name):
    if not np.isfinite(value).all():
        raise ValueError(f'non-finite {name} after numerical computation')
    return value


def residual_summary(values, expected_count, name):
    """Legacy summary filters non-finite entries; strict metrics must never do so."""
    values = np.asarray(values, dtype=float)
    if values.shape != (expected_count,):
        raise ValueError(f'{name} residual count differs from association/pair count')
    finite_array(values, name)
    result = summary(values)
    if expected_count:
        if result is None or result['n'] != expected_count:
            raise ValueError(f'{name} summary dropped residuals')
        finite_array(np.asarray(list(result.values()), dtype=float), f'{name} summary')
    return result


def rotation(q):
    q = vector(q, 4, 'quaternion wxyz')
    if abs(np.linalg.norm(q) - 1) > 1e-6:
        raise ValueError('quaternion must be unit length (tolerance 1e-6)')
    return quat_wxyz_to_matrix(q)


def transform(value, name):
    if not isinstance(value, list) or len(value) != 4:
        raise ValueError(f'{name} must be a row-major 4x4 rigid transform')
    m = np.array([vector(row, 4, name) for row in value])
    if (not np.allclose(m[3], [0, 0, 0, 1], atol=1e-9, rtol=0)
            or not np.allclose(m[:3, :3].T @ m[:3, :3], np.eye(3), atol=1e-8, rtol=0)
            or not np.isclose(np.linalg.det(m[:3, :3]), 1, atol=1e-8, rtol=0)):
        raise ValueError(f'{name} must be SE(3), scale fixed to 1')
    return m


def protocol_schedule(protocol):
    if not isinstance(protocol, dict) or protocol.get('schema') != 'slam-refact-xr-strict-protocol-v1':
        raise ValueError('explicit slam-refact-xr-strict-protocol-v1 sidecar required')
    for key in ('clock_mapping_provenance', 'metric_scale_provenance', 'ground_truth_provenance'):
        if not isinstance(protocol.get(key), str) or not protocol[key].strip():
            raise ValueError(f'{key} is required; do not infer provenance')
    if protocol.get('pose_convention') != 'world_from_camera' or protocol.get('pose_units') != 'metres':
        raise ValueError('world_from_camera poses in verified metres are required')
    if protocol.get('pose_axes') != 'right_handed' or protocol.get('trajectory_kind') != 'published_online':
        raise ValueError('right_handed published_online pose convention is required')
    if protocol.get('ground_truth_kind') not in ('independent', 'synthetic', 'reference'):
        raise ValueError('ground_truth_kind must be independent, synthetic, or reference')
    if not isinstance(protocol.get('physical_exposure_verified'), bool):
        raise ValueError('physical_exposure_verified must be explicitly true or false')
    frames = protocol.get('source_schedule')
    if not isinstance(frames, list) or not frames:
        raise ValueError('frozen full-session source_schedule is required')
    schedule = {}
    last_t = -np.inf
    for frame in frames:
        frame_id, t = integer(frame['id'], 'source id'), finite(frame['t'], 'source time in GT clock')
        if frame_id in schedule or t <= last_t:
            raise ValueError('source schedule IDs must be unique and times strictly increasing')
        schedule[frame_id] = t
        last_t = t
    if integer(protocol.get('expected_frames'), 'expected_frames') != len(schedule):
        raise ValueError('expected_frames differs from full source schedule')
    return schedule


def inspect_v3_sources(log, protocol):
    """Validate M0 source receipts without treating them as exposure or metre poses."""
    rows = log['rows']
    diag = {'schema': log['schema'], 'stored_rows': len(rows),
            'dropped_rows': integer(log.get('droppedRows'), 'droppedRows'),
            'null_updates': integer(log.get('nullUpdates'), 'nullUpdates'),
            'position_units': 'responsive_nonmetric', 'physical_exposure_verified': False,
            'source_timestamp_field': 'source.sourceVideoTimeS',
            'fresh_source_count': 0, 'unknown_source_rows': 0, 'no_pose_rows': 0,
            'old_epoch_rows': 0, 'upload_source_time_different_rows': 0,
            'clock_mapping': 'unverified_no_GT_conversion'}
    if log.get('cap') is not None and len(rows) > integer(log['cap'], 'cap'):
        raise ValueError('stored row count exceeds declared cap')
    metadata = protocol.get('row_metadata')
    schedule = None
    if metadata is not None:
        if not isinstance(metadata, list) or len(metadata) != len(rows):
            raise ValueError('v3 row_metadata must describe every recorded row')
        schedule = protocol_schedule(protocol)
        for meta in metadata:
            if not isinstance(meta, dict):
                raise ValueError('v3 row_metadata entries must be objects')
            if integer(meta.get('source_id'), 'source_id') not in schedule:
                raise ValueError('v3 row_metadata source_id is outside source schedule')
            integer(meta.get('epoch'), 'epoch')
            if not isinstance(meta.get('fresh'), bool):
                raise ValueError('v3 row_metadata fresh must be explicit')
        if protocol.get('timestamp_field') != 'source.sourceVideoTimeS':
            raise ValueError('v3 timestamp_field must be source.sourceVideoTimeS, never the uploaded frame clock')
        scale = finite(protocol.get('timestamp_scale_to_gt'), 'timestamp_scale_to_gt')
        offset = finite(protocol.get('timestamp_offset_to_gt_s'), 'timestamp_offset_to_gt_s')
        tolerance = finite(protocol.get('source_timestamp_tolerance_s'), 'source_timestamp_tolerance_s')
        if scale <= 0 or tolerance < 0:
            raise ValueError('invalid v3 declared clock mapping/tolerance')
        if protocol['physical_exposure_verified']:
            raise ValueError('v3 source receipt does not verify physical exposure')
        diag['clock_mapping'] = 'declared_schedule_consistency_only'
    seen, last_source_time, last_source_index = set(), {}, {}
    previous_epoch, previous_current_epoch, previous_publication = -1, -1, -np.inf
    flag_keys = ('hasPose', 'fresh', 'engineFresh', 'newEngineResponse', 'repeated',
                 'publicationRepeated', 'sourceMetadataFound')
    for index, row in enumerate(rows):
        if not isinstance(row, dict):
            raise ValueError('v3 rows must be objects')
        publication = finite(row.get('tPerfMs'), 'tPerfMs')
        if publication < previous_publication:
            raise ValueError('v3 publication timestamps must be nondecreasing')
        previous_publication = publication
        source = row.get('source')
        if source is None:
            diag['unknown_source_rows'] += 1
            if metadata is not None and metadata[index].get('fresh') is not False:
                raise ValueError('unknown v3 source cannot be promoted to fresh')
            continue
        if not isinstance(source, dict):
            raise ValueError('v3 source must be an object or null')
        if source.get('kind') not in (None, 'no-result', 'engine-response', 'callback-repeat', 'held-gap', 'inactive-held', 'held'):
            raise ValueError('unknown v3 source kind')
        for key in flag_keys:
            if source.get(key) is not None and not isinstance(source[key], bool):
                raise ValueError(f'v3 source.{key} must be boolean or null')
        if source.get('kind') == 'no-result' and (
                any(source.get(key) is True for key in ('hasPose', 'sourceMetadataFound', 'fresh'))
                or any(source.get(key) is not None for key in ('sourceFrameIndex', 'sessionEpoch',
                    'sourceVideoTimeS', 'sourceFrameTimeS', 'sourceStageTimeNanos', 'sourceTimeNanos'))):
            raise ValueError('v3 no-result contradicts pose/source metadata/freshness')
        ints = {key: None if source.get(key) is None else integer(source[key], key)
                for key in ('sourceFrameIndex', 'sessionEpoch', 'currentSessionEpoch', 'sourceTimeNanos')}
        if ints['currentSessionEpoch'] is not None:
            if ints['currentSessionEpoch'] < previous_current_epoch:
                raise ValueError('v3 current session epoch cannot decrease')
            previous_current_epoch = ints['currentSessionEpoch']
        times = {key: None if source.get(key) is None else finite(source[key], key)
                 for key in ('sourceVideoTimeS', 'sourceFrameTimeS', 'sourceStageTimeNanos',
                             'callbackVideoTimeS', 'callbackFrameTimeS', 'engineReturnedAtMs')}
        for field in ('videoTimeS', 'frameTimeS'):
            callback = 'callback' + field[0].upper() + field[1:]
            if row.get(field) is not None and times[callback] is not None and finite(row[field], field) != times[callback]:
                raise ValueError(f'v3 row {field} contradicts source.{callback}')
        has_pose, found = source.get('hasPose') is True, source.get('sourceMetadataFound') is True
        if not found:
            diag['unknown_source_rows'] += 1
        current = ints['sessionEpoch'] is not None and ints['sessionEpoch'] == ints['currentSessionEpoch']
        fresh = source.get('fresh') is True
        if fresh and not (has_pose and found and current and source.get('repeated') is False
                          and source.get('publicationRepeated') is False and source.get('kind') not in (None, 'no-result')):
            raise ValueError('v3 fresh contradicts pose/source metadata/epoch/repeated flags')
        if source.get('hasPose') is False:
            diag['no_pose_rows'] += 1
        if has_pose and ints['sessionEpoch'] is not None and not current:
            diag['old_epoch_rows'] += 1
        if found:
            if not has_pose or ints['sourceFrameIndex'] is None or ints['sessionEpoch'] is None or times['sourceVideoTimeS'] is None:
                raise ValueError('v3 matched source metadata is incomplete')
            if row.get('sourceFrameIndex') is not None and integer(row['sourceFrameIndex'], 'sourceFrameIndex') != ints['sourceFrameIndex']:
                raise ValueError('v3 legacy sourceFrameIndex contradicts pose sourceFrameIndex')
            expected_ns = times['sourceFrameTimeS'] * 1e9 if times['sourceFrameTimeS'] else times['sourceStageTimeNanos']
            if expected_ns is None or not np.isfinite(expected_ns) or ints['sourceTimeNanos'] != int(expected_ns):
                raise ValueError('v3 sourceTimeNanos contradicts its staged source time')
            if times['callbackVideoTimeS'] is not None and times['callbackVideoTimeS'] != times['sourceVideoTimeS']:
                diag['upload_source_time_different_rows'] += 1
        if fresh:
            key = (ints['sessionEpoch'], ints['sourceFrameIndex'])
            if key in seen:
                raise ValueError('duplicate v3 fresh source tuple')
            if (key[0] < previous_epoch or key[1] <= last_source_index.get(key[0], -1)
                    or times['sourceVideoTimeS'] <= last_source_time.get(key[0], -np.inf)):
                raise ValueError('v3 fresh epoch/source timestamps must be chronological')
            seen.add(key)
            last_source_time[key[0]], previous_epoch = times['sourceVideoTimeS'], key[0]
            last_source_index[key[0]] = key[1]
            diag['fresh_source_count'] += 1
        if metadata is not None:
            meta = metadata[index]
            if not isinstance(meta, dict):
                raise ValueError('v3 row_metadata entries must be objects')
            frame_id, epoch = integer(meta.get('source_id'), 'source_id'), integer(meta.get('epoch'), 'epoch')
            if frame_id not in schedule or not isinstance(meta.get('fresh'), bool):
                raise ValueError('v3 row metadata needs source schedule identity and explicit fresh')
            if meta['fresh'] != fresh:
                raise ValueError('v3 row_metadata fresh contradicts source freshness')
            if ints['sourceFrameIndex'] is not None and frame_id != ints['sourceFrameIndex']:
                raise ValueError('v3 sourceFrameIndex contradicts row_metadata.source_id')
            if ints['sessionEpoch'] is not None and epoch != ints['sessionEpoch']:
                raise ValueError('v3 source epoch contradicts row_metadata epoch')
            if found:
                mapped = finite(times['sourceVideoTimeS'] * scale + offset, 'mapped pose source timestamp')
                if abs(mapped - schedule[frame_id]) > tolerance:
                    raise ValueError('v3 mapped pose time differs from source schedule')
    return diag


def phone_rows(document, protocol):
    """Read v1 metric annotations; recognize v2/v3 for fail-closed diagnostics.

    row_metadata is a one-to-one annotation produced from source instrumentation,
    not a license to invent frame identities from callback order.
    """
    if not isinstance(document, dict) or not isinstance(protocol, dict):
        raise ValueError('recording and protocol must be JSON objects')
    log = document.get('poseLog') if document.get('schema') == 'slam-refact-phone-metrics-v1' else document
    if not isinstance(log, dict) or log.get('schema') not in ('slam-refact-xr-poselog-v1', 'slam-refact-xr-poselog-v2', 'slam-refact-xr-poselog-v3'):
        raise ValueError('slam-refact-xr-poselog-v1, v2 or v3 poseLog required')
    is_v2 = log['schema'] == 'slam-refact-xr-poselog-v2'
    is_v3 = log['schema'] == 'slam-refact-xr-poselog-v3'
    if (is_v2 or is_v3) and not isinstance(log.get('meta'), dict):
        raise ValueError('v2/v3 meta object is required; its prose does not establish metric calibration')
    raw = log.get('rows')
    if not isinstance(raw, list):
        raise ValueError('poseLog.rows must be an array')
    if is_v3:
        diagnostics = inspect_v3_sources(log, protocol)
        raise DiagnosticOnly('v3 is nonmetric; source timing diagnostics do not establish metre scale or exposure', diagnostics)
    metadata = protocol.get('row_metadata')
    if not isinstance(metadata, list) or len(metadata) != len(raw):
        raise ValueError('explicit row_metadata for every stored row is required (source_id, fresh, epoch)')
    schedule = protocol_schedule(protocol) if is_v2 else None
    dropped = integer(log.get('droppedRows'), 'droppedRows')
    null_updates = integer(log.get('nullUpdates'), 'nullUpdates')
    if log.get('cap') is not None and len(raw) > integer(log['cap'], 'cap'):
        raise ValueError('stored row count exceeds declared cap')
    field = protocol.get('timestamp_field')
    if field not in ('videoTimeS', 'frameTimeS'):
        raise ValueError('timestamp_field must be videoTimeS or frameTimeS; tPerfMs is not source time')
    scale = finite(protocol.get('timestamp_scale_to_gt'), 'timestamp_scale_to_gt')
    offset = finite(protocol.get('timestamp_offset_to_gt_s'), 'timestamp_offset_to_gt_s')
    if scale <= 0:
        raise ValueError('timestamp scale must be positive')
    rows, previous_publication = [], -np.inf
    seen_fresh, last_source_time, last_epoch = set(), -np.inf, -1
    for row, meta in zip(raw, metadata):
        if not isinstance(row, dict) or not isinstance(meta, dict):
            raise ValueError('pose rows and row_metadata entries must be objects')
        if is_v2:
            frame_id = integer(meta.get('source_id'), 'source_id')
            epoch = integer(meta.get('epoch'), 'epoch')
            if frame_id not in schedule:
                raise ValueError('v2 source_id is outside the frozen schedule')
            if not isinstance(meta.get('fresh'), bool):
                raise ValueError('v2 fresh must be explicit; repeatFrame does not determine pose freshness')
            source_index = row.get('sourceFrameIndex')
            if source_index is not None and integer(source_index, 'sourceFrameIndex') != frame_id:
                raise ValueError('v2 sourceFrameIndex contradicts row_metadata.source_id')
            if row.get('repeatFrame') is not None and not isinstance(row['repeatFrame'], bool):
                raise ValueError('v2 repeatFrame must be boolean or null')
            if meta['fresh']:
                if frame_id in seen_fresh or schedule[frame_id] <= last_source_time:
                    raise ValueError('v2 fresh source IDs must be unique and chronological')
                if epoch < last_epoch:
                    raise ValueError('v2 fresh epoch cannot decrease')
                seen_fresh.add(frame_id)
                last_source_time, last_epoch = schedule[frame_id], epoch
        publication = finite(row.get('tPerfMs'), 'tPerfMs')
        if publication < previous_publication:
            raise ValueError('publication timestamps must be nondecreasing')
        previous_publication = publication
        if is_v2:
            continue  # Uploaded-frame clocks are not converted into pose-source times.
        p, q = row.get('position'), row.get('rotation')
        rows.append({
            'source_id': meta['source_id'], 'fresh': meta['fresh'], 'epoch': meta['epoch'],
            't': finite(row.get(field), field) * scale + offset,
            'status': row.get('status'),
            'p': None if p is None else [p['x'], p['y'], p['z']],
            'q_wxyz': None if q is None else [q['w'], q['x'], q['y'], q['z']],
        })
    if is_v2:
        # This is the CURRENT v2 schema contract, not interpretation of meta prose.
        # Its timestamps belong to the uploaded frame; the pose source lags it.
        # A sidecar declaration cannot change units or invent the missing source time.
        raise ValueError('v2 is nonmetric and lacks pose-source timestamps; metric export contract required')
    return rows, {'stored_rows': len(raw), 'dropped_rows': dropped, 'null_updates': null_updates,
                  'recording_complete': dropped == 0,
                  'limitations': 'pose-only observation; no image/IMU replay or inferred exposure timing'}


def associate(times, gt_times, max_dt):
    """Closest-time-first one-to-one matching, deterministic ties; reject crossings."""
    candidates = []
    for i, t in enumerate(times):
        lo, hi = np.searchsorted(gt_times, [t - max_dt, t + max_dt], side='left')
        while hi < len(gt_times) and gt_times[hi] <= t + max_dt:
            hi += 1
        candidates.extend((abs(t - gt_times[j]), i, j) for j in range(lo, hi))
    used_i, used_j, pairs = set(), set(), []
    for _, i, j in sorted(candidates):
        if i not in used_i and j not in used_j:
            pairs.append((i, j))
            used_i.add(i)
            used_j.add(j)
    pairs.sort()
    if any(b[1] <= a[1] for a, b in zip(pairs, pairs[1:])):
        raise ValueError('ambiguous crossing GT association; tighten registered time tolerance')
    return pairs


def evaluate(rows, gt_rows, protocol):
    """Compute one-sequence metrics; thresholds, paired arms and final gates are not implemented."""
    try:
        with np.errstate(over='raise', invalid='raise', divide='raise'):
            return _evaluate(rows, gt_rows, protocol)
    except (FloatingPointError, OverflowError, np.linalg.LinAlgError) as exc:
        raise ValueError(f'non-finite or failed numerical computation: {exc}') from exc


def _evaluate(rows, gt_rows, protocol):
    schedule = protocol_schedule(protocol)
    max_dt = finite(protocol.get('association_max_dt_s'), 'association_max_dt_s')
    source_dt = finite(protocol.get('source_timestamp_tolerance_s'), 'source_timestamp_tolerance_s')
    rpe_dt = finite(protocol.get('rpe_tolerance_s'), 'rpe_tolerance_s')
    if min(max_dt, source_dt, rpe_dt) < 0:
        raise ValueError('timestamp tolerances must be nonnegative')
    statuses = protocol.get('accepted_statuses')
    if not isinstance(statuses, list) or not statuses or any(not isinstance(x, str) for x in statuses):
        raise ValueError('explicit accepted_statuses are required')
    t_bc = transform(protocol.get('T_body_camera'), 'T_body_camera')
    accepted, excluded, fresh_ids = [], [], set()
    previous_fresh_time, previous_epoch = -np.inf, -1
    for row_index, row in enumerate(rows):
        frame_id = integer(row.get('source_id'), 'source_id')
        epoch = integer(row.get('epoch'), 'epoch')
        if frame_id not in schedule:
            raise ValueError('row source_id is outside the frozen schedule')
        t = finite(row.get('t'), 'row source timestamp')
        if abs(t - schedule[frame_id]) > source_dt:
            raise ValueError('row timestamp differs from declared source frame')
        if not isinstance(row.get('fresh'), bool):
            raise ValueError('fresh must be explicitly true or false')
        if not row['fresh']:
            excluded.append({'row': row_index, 'source_id': frame_id, 'reason': 'repeated'})
            continue
        if frame_id in fresh_ids or t <= previous_fresh_time:
            raise ValueError('fresh source IDs/timestamps must be unique and chronological')
        if epoch < previous_epoch:
            raise ValueError('fresh epoch cannot decrease')
        fresh_ids.add(frame_id)
        previous_fresh_time, previous_epoch = t, epoch
        if row.get('status') not in statuses:
            excluded.append({'row': row_index, 'source_id': frame_id, 'reason': 'status', 'status': row.get('status')})
            continue
        if row.get('p') is None and row.get('q_wxyz') is None:
            excluded.append({'row': row_index, 'source_id': frame_id, 'reason': 'missing_pose'})
            continue
        r_wc, p_wc = rotation(row.get('q_wxyz')), vector(row.get('p'), 3, 'position')
        r_wb = finite_array(r_wc @ t_bc[:3, :3].T, 'body rotation')
        p_wb = finite_array(p_wc - r_wb @ t_bc[:3, 3], 'body position')
        accepted.append((frame_id, t, r_wb, p_wb, epoch))
    gt = np.array([vector(row, 8, 'GT t xyz qxyzw') for row in gt_rows])
    if len(gt) < 3 or not np.all(np.diff(gt[:, 0]) > 0):
        raise ValueError('GT needs at least three strictly increasing unique timestamps')
    gr_all = [rotation([row[7], *row[4:7]]) for row in gt]
    pairs = associate([row[1] for row in accepted], gt[:, 0], max_dt)
    if len(pairs) < 3:
        raise ValueError('fewer than three unique GT associations; no accuracy assessment')
    matched = [accepted[i] for i, _ in pairs]
    ids = [row[0] for row in matched]
    e = np.array([row[3] for row in matched])
    er = np.array([row[2] for row in matched])
    g = np.array([gt[j, 1:4] for _, j in pairs])
    gr = np.array([gr_all[j] for _, j in pairs])
    alignment = protocol.get('alignment', {})
    if not isinstance(alignment, dict):
        raise ValueError('alignment must be an object')
    if alignment.get('type') == 'fixed':
        a = transform(alignment.get('T_gt_est'), 'T_gt_est')
        ra, ta = a[:3, :3], a[:3, 3]
    elif alignment.get('type') == 'fit_se3':
        fit_ids = alignment.get('source_ids')
        if (not isinstance(fit_ids, list) or len(fit_ids) < 3 or len(set(fit_ids)) != len(fit_ids)
                or any(frame_id not in ids for frame_id in fit_ids)):
            raise ValueError('all preregistered alignment source IDs must have unique GT matches')
        for frame_id in fit_ids:
            integer(frame_id, 'alignment source id')
        ix = [ids.index(frame_id) for frame_id in fit_ids]
        cov = finite_array((g[ix] - g[ix].mean(0)).T @ (e[ix] - e[ix].mean(0)), 'alignment covariance')
        if np.linalg.matrix_rank(cov) < 2:
            raise ValueError('position-based SE3 rotation alignment is unobservable; no fallback alignment')
        ra, ta, _ = umeyama(e[ix], g[ix], with_scale=False)
    else:
        raise ValueError('alignment must be fixed or fit_se3 with preregistered source_ids')
    finite_array(ra, 'alignment rotation')
    finite_array(ta, 'alignment translation')
    aligned = finite_array((ra @ e.T).T + ta, 'aligned positions')
    aligned_r = finite_array(ra @ er, 'aligned rotations')
    ate_t = np.linalg.norm(finite_array(aligned - g, 'ATE residual vectors'), axis=1)
    ate_r = [rotation_angle_deg(rr.T @ ee) for rr, ee in zip(gr, aligned_r)]
    ts = np.array([row[1] for row in matched])
    rpe_t, rpe_r, rpe_pairs = [], [], []
    for i, t in enumerate(ts):
        k = np.searchsorted(ts, t + 1.0)
        options = [j for j in (k - 1, k) if i < j < len(ts)]
        if not options:
            continue
        j = min(options, key=lambda n: (abs(ts[n] - t - 1.0), n))
        if abs(ts[j] - t - 1.0) > rpe_dt:
            continue
        delta_r_gt = gr[i].T @ gr[j]
        delta_r_est = er[i].T @ er[j]
        delta_t_gt = finite_array(gr[i].T @ (g[j] - g[i]), 'GT relative translation')
        delta_t_est = finite_array(er[i].T @ (e[j] - e[i]), 'estimated relative translation')
        residual_t = finite_array(delta_r_gt.T @ (delta_t_est - delta_t_gt), 'RPE residual vector')
        rpe_t.append(np.linalg.norm(residual_t))
        rpe_r.append(rotation_angle_deg(delta_r_gt.T @ delta_r_est))
        rpe_pairs.append([ids[i], ids[j]])
    matched_set = set(ids)
    return {
        'schema': 'slam-refact-xr-strict-metrics-v1', 'assessment': 'metrics_only_not_acceptance',
        'ground_truth_kind': protocol['ground_truth_kind'],
        'physical_exposure_verified': protocol['physical_exposure_verified'],
        'alignment': {'type': alignment['type'], 'scale': 1.0, 'R': ra.tolist(), 't': ta.tolist(),
                      'source_ids': alignment.get('source_ids'), 'rotation_uses_same_SE3': True},
        'expected_frames': len(schedule), 'fresh_rows': len(fresh_ids), 'valid_poses': len(accepted),
        'associated_poses': len(pairs), 'full_session_pose_coverage': len(accepted) / len(schedule),
        'full_session_associated_coverage': len(pairs) / len(schedule),
        'missing_source_ids': [i for i in schedule if i not in fresh_ids],
        'unassociated_source_ids': [row[0] for row in accepted if row[0] not in matched_set],
        'excluded_rows': excluded, 'associations': [{'source_id': accepted[i][0], 'gt_index': j,
            'dt_s': float(accepted[i][1] - gt[j, 0])} for i, j in pairs],
        'ate_translation_m': residual_summary(ate_t, len(pairs), 'ATE translation'),
        'ate_rotation_deg': residual_summary(ate_r, len(pairs), 'ATE rotation'),
        'rpe_1s': {'pairs': rpe_pairs,
                   'translation_m': residual_summary(rpe_t, len(rpe_pairs), 'RPE translation'),
                   'rotation_deg': residual_summary(rpe_r, len(rpe_pairs), 'RPE rotation')},
        'metric_complete': bool(rpe_pairs),
        'limitations': ['No metric threshold or product verdict', 'No paired-arm/macro/locked-final gate',
                       'Cross-epoch errors retained; no per-epoch alignment',
                       'Provenance declarations must be independently verified'],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recording', required=True, type=Path)
    parser.add_argument('--protocol', required=True, type=Path)
    parser.add_argument('--ground-truth', required=True, type=Path, help='JSON array: t x y z qx qy qz qw; body poses')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    try:
        protocol = json.loads(args.protocol.read_text())
        rows, recording = phone_rows(json.loads(args.recording.read_text()), protocol)
        result = evaluate(rows, json.loads(args.ground_truth.read_text()), protocol)
        result['recording'] = recording
        result['inputs'] = {name: {'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
                            for name, path in [('recording', args.recording), ('protocol', args.protocol),
                                               ('ground_truth', args.ground_truth)]}
        code = 0 if result['metric_complete'] and recording['recording_complete'] else 2
    except (ValueError, KeyError, TypeError, OSError) as exc:
        result, code = {'assessment': 'UNVERIFIED', 'error': str(exc)}, 2
        if isinstance(exc, DiagnosticOnly):
            result['diagnostics'] = exc.diagnostics
    output = json.dumps(result, indent=2, allow_nan=False) + '\n'
    if args.output:
        with args.output.open('x') as stream:
            stream.write(output)
    print(output, end='')
    return code


if __name__ == '__main__':
    raise SystemExit(main())
