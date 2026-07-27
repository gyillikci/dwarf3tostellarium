"""
Catalog of DWARF3's on-device NPU (Artosyn AR9341) AI models, found directly
from the firmware's config/config/*.json files (see memory:
dwarf3-npu-architecture.md, "complete NPU model catalog found directly").

Each model is loaded at runtime from /usrdata/model/arnn_model/<file> on the
device -- only match_cnn.npubin ships in the particular OTA delta package
that was examined; the rest are presumably provisioned separately (factory
image, or downloaded) since they weren't present in either available OTA
version.

This is reference data, not executable inference code -- there's no way to
run these models without the actual .npubin weight files, which were not
obtainable (see dwarf3-npu-architecture.md for why: absent from both OTA
versions, no live-device shell access, FTP jailed to SD-card photo storage
only).
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional


@dataclass
class NpuModel:
    config_name: str          # the config/config/<name>.json filename (no extension)
    device_filename: str      # the actual .npubin filename referenced inside that json
    purpose: str              # what this model does, in plain terms
    confirmed_wscmd: Optional[str] = None   # the WsCmd this model backs, if known
    classes: List[str] = field(default_factory=list)  # detector class list, if applicable
    notes: str = ""


NPU_MODELS: Dict[str, NpuModel] = {
    "astro_denoise": NpuModel(
        config_name="astro_denoise",
        device_filename="astro_denoise.npubin",
        purpose="AI-based image denoising",
        confirmed_wscmd="CMD_ASTRO_START_AI_ENHANCE / CMD_ASTRO_STOP_AI_ENHANCE (11029/11030)",
        notes='Internal network name is literally "ai_astro_denoise" -- this IS "AI Enhance". '
              "The dispatch handler for the START command was never located in the WsCmd "
              "table, but the model identity is certain from the JSON config name.",
    ),
    "autofocus_actor": NpuModel(
        config_name="autofocus_actor",
        device_filename="Autofocus_Actor.npubin",
        purpose="Reinforcement-learning actor network for astro autofocus (picks a focus-adjustment action)",
        confirmed_wscmd="CMD_FOCUS_START_ASTRO_AUTO_FOCUS (15004)",
        notes="Paired with autofocus_critic below -- astro autofocus is RL-based, "
              "not classical contrast-detection.",
    ),
    "autofocus_critic": NpuModel(
        config_name="autofocus_critic",
        device_filename="Autofocus_Critic.npubin",
        purpose="Reinforcement-learning critic network for astro autofocus (evaluates focus quality)",
        confirmed_wscmd="CMD_FOCUS_START_ASTRO_AUTO_FOCUS (15004)",
    ),
    "sharpness": NpuModel(
        config_name="sharpness",
        device_filename="sharpness.npubin",
        purpose="NPU-based sharpness/focus-quality scoring",
        notes="Distinct from the OpenCV unsharp-mask 'imgSharpen' post-processing filter "
              "(see astro_processing.py) -- this is a learned quality metric, likely feeding "
              "the autofocus critic or frame-selection logic, not an image filter itself.",
    ),
    "lightfc_backbone": NpuModel(
        config_name="lightfc_backbone",
        device_filename="lightfc_template_encoder_mobile.npubin",
        purpose="Siamese-tracking template encoder (LightTrack/SiamFC-family architecture)",
        notes="Paired with lightfc_search below -- template+search split is the standard "
              "architecture for real-time visual object tracking. Almost certainly the real "
              "engine behind click-to-track / MOT.",
    ),
    "lightfc_search": NpuModel(
        config_name="lightfc_search",
        device_filename="lightfc_search_network_mobile.npubin",
        purpose="Siamese-tracking search network (finds the tracked template in each new frame)",
    ),
    "track_cnn": NpuModel(
        config_name="track_cnn",
        device_filename="LFC_track.npubin",
        purpose="A second, related tracking model (same 'LFC' naming as lightfc_*)",
    ),
    "match_cnn": NpuModel(
        config_name="match_cnn",
        device_filename="match_cnn.npubin",
        purpose="Feature/template matching",
        notes="The one model actually bundled in the examined OTA's config/arnn_model/. "
              "Purpose beyond 'matching' not narrowed further.",
    ),
    "salient_seg": NpuModel(
        config_name="salient_seg",
        device_filename="salient_seg.npubin",
        purpose="Salient-object segmentation (automatic 'what is the main subject' + mask)",
    ),
    "skydetectsegment": NpuModel(
        config_name="skydetectsegment",
        device_filename="skydetectsegment.npubin",
        purpose="Sky vs. foreground segmentation",
    ),
    "tele_scene_detect": NpuModel(
        config_name="tele_scene_detect",
        device_filename="TeleSceneDetection.npubin",
        purpose="Automatic scene/shooting-mode classification for the tele camera",
    ),
    "wide_scene_detect": NpuModel(
        config_name="wide_scene_detect",
        device_filename="WideSceneDetection.npubin",
        purpose="Automatic scene/shooting-mode classification for the wide camera",
    ),
    "ufosegment": NpuModel(
        config_name="ufosegment",
        device_filename="ufosegment.npubin",
        purpose="Dedicated segmentation for UFO-tracking mode (night-tuned variant)",
    ),
    "ufosegment_day": NpuModel(
        config_name="ufosegment_day",
        device_filename="ufosegment_day.npubin",
        purpose="Dedicated segmentation for UFO-tracking mode (day-tuned variant)",
    ),
    "bird": NpuModel(
        config_name="bird",
        device_filename="YOLOv8sDet_opt_bird.npubin",
        purpose="Dedicated single-class bird detector",
        classes=["bird"],
        notes="post_process: conf=0.25, iou=0.6, class_num=1.",
    ),
    "yolov8s_2c": NpuModel(
        config_name="yolov8s_2c",
        device_filename="yolov8s_2c.npubin",
        purpose="YOLOv8s person/face detector -- almost certainly Sentry Mode's person-detection alert",
        classes=["person", "face"],
        notes="Per-camera/per-sensitivity threshold table: wide_sensity/wide_normal/"
              "tele_sensity/tele_normal, each with its own {class: threshold} pairs. "
              "post_process: conf=0.1, iou=0.3, class_num=2.",
    ),
    "yolov8s_30c": NpuModel(
        config_name="yolov8s_30c",
        device_filename="yolov8s_30c.npubin",
        purpose="YOLOv8s general object detector for Sentry/tracking modes",
        classes=[
            "person", "face", "car", "bicycle", "motorcycle", "airplane", "helicopter",
            "F1", "Balloon", "kite", "parachute", "snake", "Ball", "surfboard", "Ship",
            "cat", "dog", "rabbit", "panda", "bird", "giraffe", "penguin", "horse",
            "sheep", "cattle", "elephant", "rat", "tiger", "lion", "bear",
        ],
        notes="Same per-camera/per-sensitivity threshold table structure as yolov8s_2c. "
              "The class list (F1 race cars, kites, parachutes, plus a real zoo/wildlife "
              "set) confirms this is tuned for wildlife/zoo photography as much as security. "
              "post_process: conf=0.1, iou=0.3, class_num=30.",
    ),
}


if __name__ == "__main__":
    for name, model in NPU_MODELS.items():
        print(f"{name:20s} -> {model.device_filename:35s} {model.purpose}")
        if model.classes:
            print(f"{'':20s}    classes: {', '.join(model.classes)}")
