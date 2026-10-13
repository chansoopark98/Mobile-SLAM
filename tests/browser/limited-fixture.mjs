/** Frozen wire specimen from the A3 backend contract; no estimator or covariance oracle. */
export function limitedPipeline(timestamp = 1, engineEpoch = 1) {
    const covariance = new Array(36).fill(0);
    for (let axis = 0; axis < 6; axis++) covariance[axis * 7] = axis < 3 ? 1e-4 : 1e-6;
    return { schema: 'mobile-slam-pipeline-v2', frameTimestamp: timestamp, engineEpoch,
        solver: { eligibleFeatures: 12, structuralSlots: 12, currentMetricFactors: 1, currentProjectiveFactors: 3,
            provisionalSlots: 7, excludedSlots: 2 },
        quality: { mode: 'projective_limited', rotationVisualSupported: true, metricTranslationSupported: false,
            translationSupport: 'imu_model_only', fallbackUsed: true, recoveryMode: true,
            currentMetricFactors: 1, currentProjectiveFactors: 3, limitedDurationS: .25,
            uncertaintyAvailable: true, uncertaintyAnchorTimestamp: timestamp - .25,
            relativeCovariance6x6: covariance,
            relativeCovarianceOrdering: 'anchor_body_position_xyz_m,right_relative_rotation_xyz_rad',
            position3SigmaM: .03, rotation3SigmaRad: .003, rotationConsistencyScore: 1,
            rotationConsistencyScoreThreshold: 16.2663, consistencyScoreIsProbabilistic: false,
            uncertaintyModel: 'imu_propagation_joint_15d', uncertaintySource: 'last_metric_raw_graph',
            visualRotationRank: 3, provisionalObservationsRetired: 0, reason: 'projective_recovery' } };
}
