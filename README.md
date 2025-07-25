# MetaCam EDU @ Inno Wing

## Usage

### Video Script

https://github.com/user-attachments/assets/4e144e53-d9cc-4892-b54e-6fb2d8c2ab73

This outputs a video file with the left camera, right camera, and LiDAR data visualized side by side. It also outputs the first frame, if only the first frame is needed and not the video, then the `--first-frame` flag can be used. The `--no-color` flag can be used to disable colorization of the point cloud based on distance from the LiDAR.

Usage:
```
python video_main.py <path_to_data_folder> [--first-frame] [--no-color]
```

Example:
```
python main.py ./data/2025-05-27_11-53-47
```

### Info Script

This outputs the different topics in the data and the format of the LiDAR data.

Usage:
```
python info_main.py <path_to_data_folder>
```

### Viewer Script

This opens a window to visualize the point cloud generated by our algorithm, or by the built-in software of the device using the `--preview` flag.

Usage:
```
python viewer_main.py <path_to_data_folder> [--preview]
```

Example:
```
python viewer_main.py ./data/2025-05-27_11-53-47 --preview
```

Controls (from [Open3D docs](https://www.open3d.org/docs/latest/tutorial/Basic/visualization.html#Function-draw_geometries)):
```
-- Mouse view control --
  Left button + drag         : Rotate.
  Ctrl + left button + drag  : Translate.
  Wheel button + drag        : Translate.
  Shift + left button + drag : Roll.
  Wheel                      : Zoom in/out.

-- Keyboard view control --
  [/]          : Increase/decrease field of view.
  R            : Reset view point.
  Ctrl/Cmd + C : Copy current view status into the clipboard.
  Ctrl/Cmd + V : Paste view status from clipboard.

-- General control --
  Q, Esc       : Exit window.
  H            : Print help message.
  P, PrtScn    : Take a screen capture.
  D            : Take a depth capture.
  O            : Take a capture of current rendering settings.
```

## Notes

- The video output seems to have something wrong, it cannot be sent through WhatsApp.
  - For now, workaround can be to use `ffmpeg` to just transcode it:
    ```
    ffmpeg -i output.mp4 transcoded.mp4
    ```
