# Safari natural-feature teleoperation

This is the default phone pose source. It uses only Safari's rear-camera
frames and a lightweight JavaScript vision pipeline; it does not require an
AprilTag, WebXR, WebAssembly, or a native app.

## Scene requirements

Point the camera at a stationary, approximately flat area containing varied
texture: text, a poster, shelves, wood grain, or a tool board. Plain walls,
glass, repeating tile patterns, moving people, and changing reflections are
poor references. Keep the reference region visible while moving the phone.

## HTTPS setup

Safari grants camera access only in a secure context. Generate a certificate:

```bash
./configure_phone_https.sh
```

Install and trust `phone_https/phone-ca.crt` on the iPhone as described by the
script. Regenerate the certificate if the Pi's Wi-Fi address changes.

## Dry-run validation

Start without motor enable flags:

```bash
python3 phone_base_teleop.py \
  --tls-cert phone_https/server.crt \
  --tls-key phone_https/server.key
```

In Safari, start the camera, measure the camera-to-reference-plane distance,
enter it in the visual settings, and tap **锁定环境**. A good lock starts with
at least 45 points and continues with at least 25 green inliers. Move slowly
and verify all six displayed pose directions before enabling the xArm.

The initial distance and horizontal field-of-view determine translation
scale. Validate scale by moving the phone a measured 100 mm. Adjust the plane
distance first, then the field-of-view if depth and sideways motion do not
share the same scale.

Tracking loss, a low RANSAC inlier ratio, or a pose jump stops the arm and
requires a new lock. Releasing the joystick also stops the arm and resets its
command reference.

The page vendors the MIT-licensed jsfeat browser library in
`static/natural/jsfeat-min.js`. It provides FAST corners, pyramidal
Lucas-Kanade flow, and RANSAC homography estimation without WebAssembly.
