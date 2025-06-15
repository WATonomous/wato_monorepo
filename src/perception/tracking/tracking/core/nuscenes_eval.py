from nuscenes.nuscenes import NuScenes
from nuscenes.eval.tracking.evaluate import TrackingEval
from nuscenes.eval.common.config import config_factory
import json
import os

cfg = config_factory('tracking_nips_2019')

# Initialize the nuScenes object
data_path = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "dataset_tracking", "nuscenes"
)
with open(os.path.join(data_path, "dataset_name.txt"), 'r') as f:
    dataset_name = f.readline().rstrip('\n')
nusc = NuScenes(
    version=dataset_name,
    dataroot=os.path.join(data_path, f"{dataset_name}_data"),
    verbose=True,
)

# Load your tracking results
res_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "track_results")
res_file = f"results{len(os.listdir(res_dir)) - 1}.json"
with open(os.path.join(res_dir, res_file), 'r') as f:
    results = json.load(f)

# Initialize the evaluator
tracker_eval = TrackingEval(
    config=cfg,
    result_path=os.path.join(res_dir, res_file),
    nusc_version="v1.0-mini",
    nusc_dataroot=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "dataset_tracking", "nuscenes"
    ),
    eval_set="mini_train",
    output_dir=os.path.join(os.path.dirname(os.path.abspath(__file__)), "test_results"),
)

# Run the evaluation
tracker_eval.evaluate()
