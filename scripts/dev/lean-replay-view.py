#!/usr/bin/env python3
"""Derive a replay-ready view of an android-lean-record.mjs capture.

The live capture starts at an estimator reset, so the first stored frames can precede the first submitted IMU
sample (the engine reports imu_missing_past for them) and the last frame can follow the last IMU sample.
replay-android-input.mjs needs a past and a future IMU bracket for every replayed frame, so this keeps the
contiguous frames inside the IMU span, renumbers them from 0 and hard-links the PGM files. The source capture
is never modified; the view records which source frames were dropped and why.
"""
import argparse
import hashlib
import json
import os
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', help='android-lean-record.mjs output directory')
    parser.add_argument('output', help='new directory for the replay view (must not exist)')
    args = parser.parse_args()
    with open(os.path.join(args.source, 'record.json'), 'rb') as handle:
        raw = handle.read()
    report = json.loads(raw)
    if report.get('recorder') != 'android-lean-record' or not report.get('completed'):
        sys.exit('source is not a completed android-lean-record capture')
    recording = report['recording']
    imu, frames = recording['imu'], recording['frames']
    first, last = imu[0][0], imu[-1][0]
    kept = [frame for frame in frames if first <= frame['timestamp'] <= last]
    if len(kept) < 2:
        sys.exit('fewer than two frames inside the IMU span')
    start = frames.index(kept[0])
    if frames[start:start + len(kept)] != kept:
        sys.exit('frames inside the IMU span are not contiguous')
    os.makedirs(args.output)
    source_data = os.path.join(args.source, 'mav0/cam0/data')
    data = os.path.join(args.output, 'mav0/cam0/data')
    os.makedirs(data)
    os.makedirs(os.path.join(args.output, 'mav0/imu0'))
    view = []
    for index, frame in enumerate(kept):
        name = f'{index:05d}.pgm'
        os.link(os.path.join(source_data, frame['file']), os.path.join(data, name))
        view.append({**frame, 'index': index, 'file': name, 'sourceIndex': frame['index'], 'sourceFile': frame['file']})
    recording = {**recording, 'frames': view, 'acceptedFrameMetadata': [dict(frame) for frame in view],
                 'counters': {**recording['counters'], 'acceptedFrames': len(view)}}
    recording['replayView'] = {'source': os.path.abspath(args.source),
                               'sourceRecordSha256': hashlib.sha256(raw).hexdigest(),
                               'droppedLeading': start, 'droppedTrailing': len(frames) - start - len(kept),
                               'reason': 'frames outside the submitted IMU span have no past/future bracket'}
    recording['completion'] = {**recording['completion'], 'replayReady': True, 'imuCoversImages': True}
    with open(os.path.join(args.output, 'mav0/cam0/data.csv'), 'x') as handle:
        handle.write('#timestamp [ns],filename\n' + ''.join(f"{round(f['timestamp'] * 1e9)},{f['file']}\n" for f in view))
    os.link(os.path.join(args.source, 'mav0/imu0/data.csv'), os.path.join(args.output, 'mav0/imu0/data.csv'))
    with open(os.path.join(args.output, 'record.json'), 'x') as handle:
        json.dump({**report, 'recording': recording}, handle, indent=2)
        handle.write('\n')
    print(json.dumps({'output': args.output, 'frames': len(view), **recording['replayView']}))


if __name__ == '__main__':
    main()
