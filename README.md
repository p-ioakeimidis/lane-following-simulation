# Lane Following with OpenCV

A Python-based lane detection and steering estimation system built using OpenCV and NumPy. The project processes video frames, detects road lane markings using computer vision techniques, estimates the vehicle's steering direction, and outputs a normalized steering value suitable for autonomous driving or robotics applications.

## Features

- 🚗 Lane line detection using Hough Transform
- 📹 Real-time video processing with OpenCV
- 🎯 Steering angle estimation from detected lane geometry
- 📏 Steering normalization for control systems
- 🔍 Region of Interest (ROI) filtering to reduce noise
- 📊 Visualization of detected lanes and predicted heading
- ⚡ Lightweight implementation without deep learning

## Pipeline

1. Read input video frame
2. Apply Gaussian blur for noise reduction
3. Convert image to HSV color space
4. Perform thresholding and edge detection (Canny)
5. Extract Region of Interest (ROI)
6. Detect line segments using Probabilistic Hough Transform
7. Average left and right lane lines
8. Compute steering angle
9. Normalize steering output
10. Display lane overlays and heading visualization

## Project Structure

```
lane_following_normalized.py    # Main lane detection and steering algorithm
```

## Requirements

Install the required dependencies:

```bash
pip install opencv-python numpy matplotlib
```

## Usage

Place your input video (currently expected as `bosch_test.mp4`) in the project directory and run:

```bash
python lane_following_normalized.py
```

Press **`q`** or **`Esc`** to exit the application.

## Output Windows

The program displays several visualization windows:

- **Frame** – Original video frame
- **New Image** – Region of Interest after edge detection
- **Line Image** – Detected lane lines
- **Heading Image** – Estimated steering direction overlay

Additionally, the console prints the normalized steering value:

```
Real_Steering: 0.12
```

where:

- `0` represents driving straight,
- negative values indicate steering left,
- positive values indicate steering right.

## Steering Estimation

The steering angle is computed from the detected lane lines:

- If both lanes are detected, the midpoint between them is used.
- If only one lane is visible, the algorithm estimates direction from that lane.
- If no lanes are detected, the previous heading is smoothed and maintained.

An exponential smoothing factor is applied to reduce steering jitter between frames.

## Techniques Used

- Gaussian Blur
- HSV Color Processing
- Canny Edge Detection
- Region of Interest Masking
- Probabilistic Hough Line Transform
- Linear Regression for Lane Averaging
- Geometric Steering Angle Calculation
- Steering Normalization

## Customization

You can adjust parameters such as:

- Canny edge thresholds
- Hough Transform settings (`minLineLength`, `maxLineGap`)
- Region of Interest polygon
- HSV threshold values
- Steering smoothing factor (`alfa`)

to better suit different cameras or road conditions.

## Possible Improvements

- Add camera calibration and perspective transformation
- Support curved lane detection
- Integrate PID steering control
- Replace classical vision with deep learning-based lane segmentation
- Process live camera feeds instead of prerecorded videos
- Improve robustness under varying lighting and weather conditions

## License

This project is provided for educational and research purposes. Feel free to modify and extend it for your own applications.
