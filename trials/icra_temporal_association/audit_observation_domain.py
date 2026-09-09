"""Reconstruct the original ROI and current-vehicle opportunity, independently."""
import numpy as np


def audit_domain(run, data):
    local = np.asarray(run['localGaussianRecords'], float).reshape(-1, 32)
    increments = np.asarray(run['localIncrementRecords'], float).reshape(-1, 12)
    lookup = {tuple(r[:4].astype(int)): r for r in increments}
    poses = np.asarray(data['positions'], float)
    excluded_vehicle = excluded_boundary = excluded_range = 0
    for row in local:
        t, n = row[:2].astype(int)
        point = row[4:6]
        distances = ((poses[:, :, t-1].T - point) ** 2).sum(1)
        boundary = abs(point[0]) <= 70.4 and abs(point[1]) <= 40
        vehicle = (distances > 9).all()
        in_range = distances[n-1] <= 1600
        current = bool(boundary and vehicle and in_range)
        expected_pd = data['pd'] if current else 0.
        actual = lookup[tuple(row[:4].astype(int))]
        assert actual[10] == expected_pd and bool(actual[7]) == current, (
            tuple(row[:4].astype(int)), 'original observation domain', expected_pd, actual[10])
        excluded_boundary += int(not boundary)
        excluded_vehicle += int(not vehicle)
        excluded_range += int(not in_range)
    return dict(checked_retained_local_predictions=len(local),
        outside_rectangle=excluded_boundary, inside_vehicle_exclusion=excluded_vehicle,
        beyond_sensor_range=excluded_range)
