# Realsense Line Follower

Here's what each parameter is doing in your current line-following + red-stop + recovery system.

## Config params:

### Black Line Detection

#### `black_threshold: 75`

Converts image to black/white.

```text
pixel < 75  -> BLACK
pixel > 75  -> WHITE
```

Lower value:

```yaml
black_threshold: 50
```

* less sensitive
* ignores dark floor

Higher value:

```yaml
black_threshold: 100
```

* more sensitive
* can detect shadows as tape

---

#### `contour_min_area: 2500`

Minimum contour area accepted as a line.

```text
area < 2500
→ ignored
```

Increase if detecting floor noise.

Decrease if tape appears small in camera.

---

#### `min_tape_width: 70`

Minimum width of detected tape contour.

```text
width < 70 px
→ reject
```

Useful for rejecting:

* cables
* shadows
* tiny black objects

---

#### `max_tape_width: 600`

Rejects extremely large black regions.

```text
width > 600
→ reject
```

Useful when camera sees:

* dark wall
* entire floor
* large shadow

---

#### `width_center_bonus: 0.5`

Reduces steering when tape occupies large image area.

Current steering:

```text
steer =
(err + curve_prediction)
*
(1 - width_center_bonus * width_ratio)
```

Higher:

```yaml
0.8
```

* smoother
* less aggressive

Lower:

```yaml
0.1
```

* sharper steering

---

# Red Stop Detection

#### `red_low_h: 0`

#### `red_high_h: 10`

First HSV red range.

```text
0° to 10°
```

---

#### `red2_low_h: 170`

#### `red2_high_h: 180`

Second HSV red range.

```text
170° to 180°
```

Needed because red wraps around HSV wheel.

---

#### `red_s_min: 100`

Minimum saturation.

Higher:

```yaml
150
```

Rejects faded red objects.

---

#### `red_v_min: 100`

Minimum brightness.

Higher:

```yaml
150
```

Rejects dark red objects.

---

#### `red_min_width: 120`

Minimum width of red marker.

```text
width < 120
→ ignore
```

---

#### `red_max_width: 700`

Maximum width of red marker.

Rejects huge red regions.

---

#### `red_min_area: 2500`

Minimum red contour area.

Rejects:

* red LEDs
* reflections
* tiny stickers

---

#### `stop_time: 5.0`

Stop duration after red detection.

```yaml
5.0
```

means:

```text
STOP
↓
wait 5 seconds
↓
continue
```

---

## Region Of Interest (ROI)

#### `line_roi_height: 0.55`

Only lower part of image is used for line detection.

```yaml
0.55
```

means:

```text
bottom 55% of image
```

Higher:

```yaml
0.75
```

* closer to rover
* more stable

Lower:

```yaml
0.30
```

* sees farther ahead
* anticipates turns

---

#### `red_roi_height: 0.20`

Only bottom 20% used for red detection.

```text
red must be near rover
```

Prevents early stopping.

---

## Steering

#### `kp: 0.25`

Most important steering parameter.

Current formula:

```text
servo =
center
-
kp * error
```

Current value:

```yaml
0.25
```

Quite aggressive.

Examples:

```yaml
0.10
```

smooth

```yaml
0.20
```

moderate

```yaml
0.35
```

very aggressive

---

#### `servo_center: 90`

Straight wheels.

```yaml
90
```

If rover drifts:

```yaml
88
```

or

```yaml
92
```

---

#### `servo_min: 0`

Maximum left steering.

---

#### `servo_max: 180`

Maximum right steering.

---

## Speed

#### `speed_fast: 120`

Straight-line speed.

Currently not fully used in your latest code because you hardcoded:

```cpp
B150
```

---

#### `speed_turn: 90`

Turning speed.

Also currently bypassed by hardcoded commands.

You should eventually replace:

```cpp
publishCommand("B150");
publishCommand("B80");
```

with:

```cpp
publishCommand("B" + std::to_string(speed_fast_));
publishCommand("B" + std::to_string(speed_turn_));
```

---

## Validation

#### `min_black_pixels: 7000`

Originally intended as minimum number of black pixels.

Your current code doesn't appear to use it anymore.

Can probably remove unless re-added.

---

#### `min_fill_ratio: 0.24`

Measures how solid contour is.

```text
fill =
contour_area
/
bounding_box_area
```

Examples:

```text
0.8 → solid tape
0.5 → acceptable
0.1 → noise
```

Current:

```yaml
0.24
```

Fairly permissive.

---

#### `max_black_percent: 0.65`

Maximum percentage of ROI that can be black.

```text
black_ratio > 65%
→ reject
```

Prevents:

* camera covered
* giant shadow
* dark floor

---

## Red Recovery

#### `red_ignore_time: 2.5`

After red stop ends:

```text
resume
↓
ignore red for 2.5s
```

Prevents repeatedly stopping on same red tape.

---

## Line Recovery

#### `line_loss_delay: 1.0`

How long line must be lost before recovery begins.

Current:

```text
line gone
↓
wait 1 second
↓
reverse recovery
```

---

#### `reverse_timeout: 30.0`

Maximum recovery duration.

Current:

```yaml
30 seconds
```

This is extremely long.

I'd recommend:

```yaml
5.0
```

or

```yaml
8.0
```

for practical use.

---

#### `reverse_step_time: 2.5`

Length of each reverse step.

Current:

```yaml
2.5 seconds
```

Meaning:

```text
reverse
↓
2.5 s
↓
stop
↓
check line
```

For indoor line following, I'd usually use:

```yaml
0.5
```

to

```yaml
1.0
```

because 2.5 seconds can move the rover a very long distance backward.

---

## Parameters I would tune first

```yaml
kp: 0.20

min_tape_width: 80

line_roi_height: 0.65

reverse_timeout: 5.0

reverse_step_time: 0.75
```

These five will have the biggest impact on real-world behavior.
