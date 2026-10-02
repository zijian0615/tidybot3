# Safari AprilTag teleoperation

This mode keeps the base controls in Safari and estimates the phone's relative
6-DoF pose from a fixed AprilTag. It does not use WebXR or a native iOS app.

## Print and mount the marker

Print `apriltag_tag36h11_id0_160mm.svg` on A4 paper at **100% / actual size**.
Measure the reference line after printing; it must be 160 mm. Mount the sheet
flat and rigidly where the iPhone's rear camera can see it throughout the
motion. Avoid glossy lamination and strong reflections.

The page currently detects `tag36h11`, ID 0. Its default size is 160 mm,
measured across the outer black square. If the printed size differs, enter the
measured value in the page before testing.

## Enable Safari camera access

Safari requires HTTPS for camera access from a LAN address. Generate a local
certificate for the Pi's current Wi-Fi address:

```bash
./configure_phone_https.sh
```

Transfer `phone_https/phone-ca.crt` to the iPhone, install the downloaded
profile, then enable it under **Settings > General > About > Certificate Trust
Settings**. The script prints the exact HTTPS URL and launch command.

Regenerate the server certificate if the Pi's Wi-Fi IP changes.

## Validate without motion

Start without either enable flag:

```bash
python3 phone_base_teleop.py \
  --tls-cert phone_https/server.crt \
  --tls-key phone_https/server.key \
  --diagnostics
```

Open the printed HTTPS URL in Safari and tap **启动相机和 AprilTag**. The green
outline must remain locked to tag ID 0, the reported distance should be close
to the measured camera-to-tag distance, and loss of the marker must report
that the arm stopped.

The horizontal field-of-view setting controls pose scale. Adjust it until the
reported distance and a measured 100 mm phone movement agree. Do this before
enabling the xArm.

After validating coordinate directions at low speed, use `--enable-xarm` and
`--enable-motors` as required. Holding the joystick or a turn button is the
shared deadman. Releasing it resets the arm's phone reference pose.

The browser detector is the BSD-licensed `apriltag-js` WebAssembly build of
AprilRobotics AprilTag 3; its license is stored in
`static/apriltag/LICENSE.apriltag-js`.
