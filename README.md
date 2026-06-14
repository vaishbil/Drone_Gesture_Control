# Drone Gesture Control

A hand gesture-based control system for the Pluto drone by [Drona Aviation](https://dronaaviation.com), developed during an internship at Drona Aviation in 2024. The system uses a webcam, a custom-trained Keras image classification model, and the Pluto's MSP (Multiwii Serial Protocol) interface to translate hand gestures into real-time drone flight commands.

---

## About the Project

This project was built during an internship at Drona Aviation (2024) as an exploration of touchless, vision-based drone control. Instead of a physical remote or a mobile app, the drone is flown entirely through hand gestures captured by a webcam.

The pipeline works in three stages: hand images are collected and labelled per gesture class, a model is trained externally using Teachable Machine, and the trained model is then loaded at runtime to classify gestures live and dispatch the corresponding flight command to the Pluto drone over WiFi.

---

## Gesture-to-Command Mapping

| Gesture | Drone Action |
|---|---|
| Arm | Arms the drone motors |
| Takeoff | Arms, box-arms, and initiates takeoff |
| Land | Lands the drone |
| Forward | Pitches forward |
| Backward | Pitches backward |
| Left | Rolls left |
| Right | Rolls right |
| Increased Height | Increases throttle |
| Decreased Height | Decreases throttle |

---

## File Structure

| File | Description |
|---|---|
| `Data Collection.py` | Captures and saves hand gesture images from a webcam, normalized to a 300x300 white canvas, for building the training dataset |
| `keras_model.h5` | Trained Keras image classification model (trained via Google Teachable Machine) |
| `labels.txt` | Class label file corresponding to the model's output indices |
| `Pluto.py` | Drone control wrapper — exposes high-level methods (`take_off`, `land`, `forward`, etc.) and handles MSP communication with the Pluto over WiFi via a background thread |
| `plutoMultiwii.py` | Low-level MSP protocol implementation for sending raw RC and command packets to the Pluto flight controller |
| `Test.py` | Main runtime script — reads webcam frames, detects hands, classifies gestures, and dispatches drone commands in real time |

---

## How It Works

**1. Data Collection**

`Data Collection.py` uses cvzone's `HandDetector` to isolate the hand in each webcam frame, crops it, and fits it onto a 300x300 white background while preserving aspect ratio. Pressing `s` saves the current frame to a labelled folder. This process is repeated for each gesture class.

**2. Model Training**

The collected images are uploaded to [Google Teachable Machine](https://teachablemachine.withgoogle.com/) to train an image classification model. The exported model (`keras_model.h5`) and its label file (`labels.txt`) are then dropped into the project directory.

**3. Real-Time Inference and Control**

`Test.py` runs the live loop:
- Reads frames from the webcam
- Detects the hand using cvzone's `HandDetector`
- Crops and normalises the hand image to 300x300
- Passes the image to the `Classifier` (cvzone wrapper around the Keras model)
- Maps the predicted label to a drone command
- Calls the corresponding method on the `pluto` object, which sends the MSP packet to the drone

**4. Drone Communication**

`Pluto.py` wraps the MSP protocol into a clean Python class. It runs a background thread that continuously sends RC values (`rcRoll`, `rcPitch`, `rcThrottle`, `rcYaw`, and AUX channels) to the Pluto at ~45Hz (every 0.022 seconds). Flight commands like takeoff and land are sent as MSP SET COMMAND packets.

---

## Tech Stack

- Python 3.10.9
- OpenCV (`cv2`) — webcam capture and frame display
- cvzone — hand detection (`HandTrackingModule`) and model inference (`ClassificationModule`)
- TensorFlow / Keras — gesture classification model
- NumPy — image array manipulation
- Google Teachable Machine — model training
- Pluto MSP protocol (`plutoMultiwii.py`) — low-level drone communication

---

## Hardware

- Pluto / Pluto X drone by [Drona Aviation](https://dronaaviation.com)
- Any standard webcam (connected to the host machine running the scripts)

---

## Setup and Usage

### Prerequisites

- Python 3.10.9
- Pluto drone connected to your machine over WiFi
- A webcam

### Install Dependencies

```bash
pip install opencv-python cvzone tensorflow keras numpy
```

### Step 1 — Collect Training Data (optional if retraining)

Open `Data Collection.py` and set the `folder` variable to the path for the gesture class you want to collect (e.g. `Data/Takeoff`). Run the script and press `s` to save each frame. Repeat for all gesture classes.

```bash
python "Data Collection.py"
```

### Step 2 — Train the Model (optional if retraining)

Upload your collected images to [Google Teachable Machine](https://teachablemachine.withgoogle.com/), train the model, and export it as a Keras model. Replace `keras_model.h5` and `labels.txt` with the exported files.

### Step 3 — Run Gesture Control

Update the model and label paths in `Test.py` if needed, then run:

```bash
python Test.py
```

Point your hand at the webcam and hold the gesture. The recognised gesture and the corresponding command will be displayed on screen, and the drone will respond in real time. Press `q` to quit.

---

## Author

Developed as an intern project at Drona Aviation (2024).

---

## License

This project is open for educational and research use. Refer to Drona Aviation's resources and the respective library licenses (TensorFlow, cvzone, OpenCV) for their usage terms.


