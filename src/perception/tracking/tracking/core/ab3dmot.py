# We implemented our method on top of AB3DMOT's KITTI tracking open-source code

from __future__ import print_function

import copy
import numpy as np
# import time

from filterpy.kalman import KalmanFilter

from .utils.config import cfg  # , cfg_from_yaml_file, log_config_to_file
from . import algorithms, covariance


def normalize_angle(angle):
    """
    Normalize angle.

    angle: In radians.
    returns: angle in radians between -pi and pi.
    """
    # reduce the angle
    twopi = (2 * np.pi)
    angle = angle % twopi

    # force it to be the positive remainder, so that 0 <= angle < 360
    angle = (angle + twopi) % twopi

    # force into the minimum absolute value residue class, so that -180 < angle <= 180
    if (angle > np.pi):
        angle -= twopi
    return angle


class KalmanBoxTracker(object):
    """This class represents the internel state of individual tracked objects observed as bbox."""

    count = 0

    def __init__(
        self, bbox3D, track_score, tracking_name, timestamp, use_angular_velocity=False
    ):
        """Initialise a tracker using initial bounding box."""
        # define constant velocity model
        # [x, y, z, rot_y, l, w, h, x_dot, y_dot, z_dot]
        if not use_angular_velocity:
            self.kf = KalmanFilter(dim_x=10, dim_z=7)
            self.kf.F = np.array([[1, 0, 0, 0, 0, 0, 0, 1, 0, 0],      # state transition matrix
                                  [0, 1, 0, 0, 0, 0, 0, 0, 1, 0],
                                  [0, 0, 1, 0, 0, 0, 0, 0, 0, 1],
                                  [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                                  [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                                  [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                                  [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                                  [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                                  [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])

            self.kf.H = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0],      # measurement function,
                                  [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                                  [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                                  [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                                  [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                                  [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                                  [0, 0, 0, 0, 0, 0, 1, 0, 0, 0]])
        else:
            # with angular velocity
            # [x, y, z, rot_y, l, w, h, x_dot, y_dot, z_dot, rot_y_dot]
            self.kf = KalmanFilter(dim_x=11, dim_z=7)
            self.kf.F = np.array([[1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],      # state transition matrix
                                  [0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                                  [0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0],
                                  [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1],
                                  [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                                  [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                                  [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                                  [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                                  [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])

            self.kf.H = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],      # measurement function,
                                  [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                  [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                                  [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                                  [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                                  [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                                  [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0]])

        # Initialize the covariance matrix, see covariance.py for more details
        cov = getattr(covariance, cfg.TRACKER.COVARIANCE)(tracking_name=tracking_name)
        self.kf.P = cov.P
        # for i in range(7):
        #     cov.Q[i, i] = 0.1
        self.kf.Q = cov.Q
        self.kf.R = cov.R
        print("Q:")
        print(cov.Q)
        print("R:")
        print(cov.R)

        self.kf.x[:7] = bbox3D.reshape((7, 1))
        # self.kf.x[7:] = np.array([0.1, 0.1, 0.1]).reshape((3, 1))  # Assuming linear velocity of 0.1  # noqa: E501

        self.last_update = timestamp
        self.id = KalmanBoxTracker.count
        KalmanBoxTracker.count += 1
        self.history = [(timestamp, self.get_state())]  # List gets updated with each update to tracker for comparison. Each track instance will have its own history List of pairs (timestamp, bbox)  # noqa: E501
        self.hits = 1           # number of total hits including the first detection
        self.hit_streak = 1     # number of continuing hit considering the first detection
        self.first_continuing_hit = 1
        self.still_first = True
        self.age = 0
        self.track_score = track_score
        self.tracking_name = tracking_name
        self.use_angular_velocity = use_angular_velocity

        # print(f"Initialized KalmanBoxTracker with state: {self.kf.x.reshape(-1)}")

    def update(self, bbox3D, info, timestamp):
        """
        Update the state vector with observed bbox.

        bbox3D: list[float] - [x, y, z, theta, l, w, h]
        info: int, float - (label, confidence score)
        timestamp: float - unix timestamp in seconds
        """
        self.last_update = timestamp
        self.hits += 1
        self.hit_streak += 1          # number of continuing hit
        if self.still_first:
            self.first_continuing_hit += 1      # number of continuing hit in the fist time

        #########################
        # Orientation Correction. Primarily aimed at cars.
        # Not so usefull for us since Perception does not give proper
        # orientation estimates for pedestrians
        #########################
        self.kf.x[3] = normalize_angle(self.kf.x[3])

        new_theta = bbox3D[3]
        new_theta = normalize_angle(new_theta)
        bbox3D[3] = new_theta

        predicted_theta = self.kf.x[3]
        angle_diff = normalize_angle(new_theta - predicted_theta)
        if abs(angle_diff) > np.pi / 2.0 and abs(angle_diff) < np.pi * 3 / 2.0:  # if the angle of two theta is not acute angle  # noqa: E501
            self.kf.x[3] += np.pi
            self.kf.x[3] = normalize_angle(self.kf.x[3])

        # now the angle is acute: < 90 or > 270, convert the case of > 270 to < 90
        if abs(new_theta - self.kf.x[3]) >= np.pi * 3 / 2.0:
            if new_theta > 0:
                self.kf.x[3] += np.pi * 2
            else:
                self.kf.x[3] -= np.pi * 2

        #########################
        # update label based on confidence
        #########################
        label, confidence = info
        if confidence > self.track_score:
            self.track_score = confidence
            self.tracking_name = label

        # if self.tracking_name in cfg.TRAFFIC_SIGN_CLASSES:
        #     self.kf.x[7:] = 0

        #########################

        self.kf.update(bbox3D)
        self.kf.x[3] = normalize_angle(self.kf.x[3])

        if (len(self.history) > 0 and self.history[-1][0] == timestamp):
            self.history[-1] = (timestamp, self.get_state())
        else:
            self.history.append((timestamp, self.get_state()))

    def predict(self, timestamp):
        """
        Advances the state vector and returns the predicted bounding box estimate.

        timestamp: unix timestamp in seconds (float)
        """
        self.kf.predict()
        self.kf.x[3] = normalize_angle(self.kf.x[3])

        self.age += 1
        if timestamp - self.last_update > 0:
            self.hit_streak = 0
            self.still_first = False
        self.history.append((timestamp, self.kf.x.reshape((-1, ))))
        return copy.copy(self.kf.x)

    def get_state(self):
        """Return the current bounding box estimate."""
        return self.kf.x.reshape((-1, ))


class AB3DMOT(object):
    def __init__(self, max_age=2, min_hits=2, tracking_name='car'):
        """
        Init.

        observation:
            - [x, y, z, rot_y, l, w, h]
        state:
            - [x, y, z, rot_y, l, w, h, x_dot, y_dot, z_dot, rot_y_dot]
        """
        self.max_age = max_age
        self.min_hits = min_hits
        self.trackers = []
        self.frame_count = 0
        self.tracking_name = tracking_name
        self.use_angular_velocity = cfg.TRACKER.USE_ANGULAR_VELOCITY
        self.match_threshold = cfg.TRACKER.MATCH_THRESHOLD
        self.score_metric_fn = getattr(algorithms, cfg.TRACKER.SCORE_METRIC)
        self.match_algorithm_fn = getattr(algorithms, cfg.TRACKER.MATCH_ALGORITHM)

    def update(self, dets, info, timestamp, update_only=False, distance_threshold=-1):
        """
        Tracker update step.

        Params:
            - dets - a numpy array of detections in the format [[x, y, z, theta, l, w, h], [x, y, z, theta, l, w, h], ...]
            - info - a numpy array of other info for each det
            - timestamp - unix timestamp in seconds (float)
            - update_only - only update tracks don't create new ones

        Requires: this method must be called once for each frame even with empty detections.
        Returns the a similar array, where the last column is the object ID.

        NOTE: The number of objects returned may differ from the number of detections provided.
        """  # noqa: E501
        '''
        Print the array of detections for debugging. Will come in the size [M, 7]
        where M is the number of detections and 7 is for [x, y, z, rot, l, w, h]
        '''

        # print(f"Detections:\n{dets}")
        for idx, detection in enumerate(dets):
            print(f"Index {idx}: {detection}")

        self.frame_count += 1

        trks = np.zeros((len(self.trackers), 7))  # N x 7 , #get predicted locations from existing trackers.  # noqa: E501
        to_del = []
        ret = []

        # Run KF Prediction on existing tracks
        for t, trk in enumerate(trks):
            pos = self.trackers[t].predict(timestamp).reshape((-1, ))
            # print(pos)
            trk[:] = [pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], pos[6]]
            if np.any(np.isnan(pos)):
                to_del.append(t)

            # print(f"deleted: {to_del}")

        trks = np.ma.compress_rows(np.ma.masked_invalid(trks))
        for t in reversed(to_del):
            self.trackers.pop(t)

        trks_S = []
        if "mahalanobis" in cfg.TRACKER.SCORE_METRIC:
            trks_S = np.array(
                [
                    np.matmul(np.matmul(tracker.kf.H, tracker.kf.P), tracker.kf.H.T)
                    + tracker.kf.R
                    for tracker in self.trackers
                ]
            )

        matched, unmatched_dets, unmatched_trks = self.associate_detections_to_trackers(
            dets, trks, trks_S=trks_S
        )
        unmatched_dets = list(unmatched_dets)

        '''
        Detections from the new frame that went unmatched and will show up in an [1, M] array where M is the number of
        unmatched detections and each detection will be represented as the index of the detections of that frame
        [1, 2, 5] = Detections 1, 2 and 5 were unmathced with current trackers
        '''  # noqa: E501

        # print(f"unmatched: {unmatched_dets}") Detections that haven't been matched

        '''
        Array of size [M, 2] where M is the number successful matches with detections and trackers and 2 is the number of indicies for each M
        [detection, tracker]
        '''  # noqa: E501
        print(f"matched: {matched}")  # Detections from the current frame that have been matched with detections from the previous frame  # noqa: E501
        print(f"unmatched tracks: {unmatched_trks}")  # Trackers that did not get an update because there were no detections that met the requirement to update the tracker  # noqa: E501

        # update matched trackers with assigned detections
        # iou_scores = None
        if distance_threshold > 0:
            distances = algorithms.euclidian_distance(dets, trks)

        for t, trk in enumerate(self.trackers):
            if t not in unmatched_trks:
                d = matched[np.where(matched[:, 1] == t)[0], 0]     # a list of index
                print(f"Processing tracker {t} with matched detection {d}")
                if len(dets[d]) > 0 and len(info[d]) > 0:
                    if distance_threshold > 0 and abs(distances[d, t]) > distance_threshold:
                        if abs(distances[d, t]) > distance_threshold * 3 and not update_only:
                            unmatched_dets = np.append(unmatched_dets, d).astype(int)
                            print(f"Added detection {d} to unmatched_dets due to distance",
                                  f"threshold. New unmatched_dets: {unmatched_dets}")
                        continue
                    else:
                        trk.update(dets[d, :][0], info[d, :][0], timestamp)
                        # Log matched tracker updates
                        print(f"Updated tracker {t} with detection {d[0]}")

        # Create and initialise new trackers for unmatched detections
        if not update_only:
            for i in unmatched_dets:        # a scalar of index
                print(f"Creating new tracker for unmatched detection {i}")
                tracking_name = info[i][0]  # class label
                track_score = info[i][1]    # confidence
                trk = KalmanBoxTracker(
                    dets[i, :],
                    track_score,
                    tracking_name,
                    timestamp,
                    self.use_angular_velocity,
                )
                self.trackers.append(trk)

        # Detect and delete old tracks
        # For each tracker, abs(timestamp - trk.last_update) calculates the number of frames since the track was last updated.  # noqa: E501
        # The condition abs(timestamp - trk.last_update) < self.max_age checks if the number of frames since the last update is less than max_age.  # noqa: E501
        # If this condition is true, the tracker is retained; otherwise, it is removed.

        print(f"Trackers before deletion: {[trk.id for trk in self.trackers]}")  # List of active trackers  # noqa: E501
        # for trk in self.trackers:
        # print(f"Tracker ID: {trk.id}, last_update: {trk.last_update}, current timestamp: {timestamp}")  # noqa: E501

        self.trackers = [
            trk
            for trk in self.trackers
            if abs(timestamp - trk.last_update) < self.max_age
        ]
        print(f"Trackers after deletion: {[trk.id for trk in self.trackers]}")  # List of trackers that were deleted because they hit the max age  # noqa: E501

        for trk in self.trackers:
            d = trk.get_state()
            # print(f"d shape: {d.shape}, trk.id: {trk.id}, trk.track_score: {trk.track_score}")

            # Checks if the trackers has been observed for at least 'min_hits' frames (trk.hits >= self.min_hits) or if it is still within the initial few frames (self.frame_count <= self.min_hits).  # noqa: E501
            # If the condition is satisfied, the tracker is included in the ret list, which contains the current tracks to be returned.  # noqa: E501
            if trk.hits >= self.min_hits or self.frame_count <= self.min_hits:
                ret.append(np.concatenate([d, [trk.id+1, trk.track_score]]))  # Concatenate d with the additional values  # noqa: E501

        if len(ret) > 0:
            return ret      # x, y, z, theta, l, w, h, ID, other info, confidence

        return np.empty((0, 15 + 7))

    def associate_detections_to_trackers(self, detections, trackers, **kwargs):
        """
        Assign detections to tracked object (both represented as bounding boxes).

        detections:  N x 7
        trackers:    M x 8
        kwargs: {
            trks_S: N x 7 x
            ...
        }

        Returns 3 lists of matches [M, 2], unmatched_detections [?] and unmatched_trackers [?]
        """
        if len(trackers) == 0:
            return (
                np.empty((0, 2), dtype=int),
                np.arange(len(detections)),
                np.empty((0, 8, 3), dtype=int),
            )
        # score matrix between detections and trackers. Lower is better. Either Mahalonobis distance or - IOU  # noqa: E501
        score_matrix = np.zeros((len(detections), len(trackers)), dtype=np.float32)

        score_matrix = self.score_metric_fn(detections, trackers, **kwargs)
        matched_indices = self.match_algorithm_fn(score_matrix)

        print(f"Matched indices before filtering: {matched_indices}")

        # Print when the matching algorithm is called
        print(f"Calling matching algorithm: {self.match_algorithm_fn.__name__}")

        unmatched_detections = []
        for d, det in enumerate(detections):
            if d not in matched_indices[:, 0]:
                unmatched_detections.append(d)
                # 2+"abc"
        unmatched_trackers = []
        for t, trk in enumerate(trackers):
            if len(matched_indices) == 0 or (t not in matched_indices[:, 1]):
                unmatched_trackers.append(t)

        # filter out matched with high score (bad)
        matches = []
        for m in matched_indices:
            match = score_matrix[m[0], m[1]] < self.match_threshold
            if not match:
                unmatched_detections.append(m[0])
                # print("ZZZZ:")
                # print(matched_indices)
                # [][0]
                unmatched_trackers.append(m[1])
            else:
                matches.append(m.reshape(1, 2))
        if len(matches) == 0:
            matches = np.empty((0, 2), dtype=int)
        else:
            matches = np.concatenate(matches, axis=0)

        return matches, np.array(unmatched_detections), np.array(unmatched_trackers)
