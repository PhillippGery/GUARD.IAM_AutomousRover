import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from huggingface_hub import HfApi

MERGE_JOBS = {
    "nadslab/guardian_strawberry_pick_all_130": [
        "nadslab/guardian_strawberry_pick_batch7",
        "nadslab/guardian_strawberry_pick_batch8",
        "nadslab/guardian_strawberry_pick_batch9",
    ],
    "nadslab/guardian_strawberry_pick_89": [
        "nadslab/guardian_strawberry_pick_batch8",
        "nadslab/guardian_strawberry_pick_batch9",
    ],
}

EXCLUDE_KEYS = {"index", "episode_index", "frame_index", "timestamp", "task_index"}

def merge(merged_repo_id, source_repos):
    print(f"\n=== Building {merged_repo_id} from {source_repos} ===")
    first = LeRobotDataset(source_repos[0], revision="main", video_backend="pyav")

    merged = LeRobotDataset.create(
        repo_id=merged_repo_id,
        fps=first.fps,
        features=first.features,
        robot_type=getattr(first.meta, "robot_type", None),
        use_videos=True,
    )

    total_episodes = 0
    for repo in source_repos:
        src = LeRobotDataset(repo, revision="main", video_backend="pyav")
        print(f"Merging {repo}: {src.num_episodes} episodes")
        for ep in src.meta.episodes:
            start = ep["dataset_from_index"]
            end = ep["dataset_to_index"]
            task = ep["tasks"][0]
            for i in range(start, end):
                frame = src[i]
                frame_dict = {}
                for k, v in frame.items():
                    if k in EXCLUDE_KEYS:
                        continue
                    if k.startswith("observation.images."):
                        v = v.permute(1, 2, 0).numpy()
                        if v.dtype != np.uint8:
                            v = (v * 255).astype(np.uint8)
                    frame_dict[k] = v
                frame_dict["task"] = task
                merged.add_frame(frame_dict)
            merged.save_episode()
            total_episodes += 1

    merged.finalize()
    print(f"Done. Merged {total_episodes} episodes into {merged_repo_id}")

    api = HfApi()
    api.create_repo(repo_id=merged_repo_id, repo_type="dataset", exist_ok=True)
    api.upload_folder(
        folder_path=str(merged.root),
        repo_id=merged_repo_id,
        repo_type="dataset",
    )
    print(f"Pushed {merged_repo_id} to Hub.")

if __name__ == "__main__":
    for merged_id, sources in MERGE_JOBS.items():
        merge(merged_id, sources)
