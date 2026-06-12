# Lane Following with OpenCV

A Python-based lane detection and steering estimation system built using OpenCV and NumPy. The project processes video frames, detects road lane markings using computer vision techniques, estimates the vehicle's steering direction, and outputs a normalized steering value suitable for autonomous driving or robotics applications.

## Input Video

This implementation processes a prerecorded **MP4 video** as its input source rather than a live camera feed. The sample video used for testing (`bosch_test.mp4`) is included in this GitHub repository, allowing users to reproduce the results and experiment with the lane detection and steering estimation pipeline immediately after setup.

If you would like to use your own video, simply replace the input file or update the path passed to `cv.VideoCapture()` in the source code.



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

# Installation

> **Note:** The instructions below are for **Windows**.

## Windows

1. Clone the repository:
   ```bash
   git clone https://github.com/your-username/your-repository.git
   cd your-repository
   ```

2. (Optional) Create and activate a virtual environment:
   ```bash
   python -m venv venv
   venv\Scripts\activate
   ```

3. Install the required Python packages:
   ```bash
   pip install opencv-python numpy matplotlib
   ```

4. Run the application:
   ```bash
   python lane_following_normalized.py
   ```

---

## Ubuntu

To run the project on Ubuntu, first install the required system packages:

```bash
sudo apt update
sudo apt install python3 python3-pip python3-venv
```

Clone the repository:

```bash
git clone https://github.com/your-username/your-repository.git
cd your-repository
```

Create and activate a virtual environment (recommended):

```bash
python3 -m venv venv
source venv/bin/activate
```

Install the Python dependencies:

```bash
pip install opencv-python numpy matplotlib
```

Run the program:

```bash
python3 lane_following_normalized.py
```

> **Note:** If you encounter issues displaying OpenCV windows on Ubuntu, make sure your system has the necessary GUI libraries installed. On desktop Ubuntu installations, these are typically available by default.
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
