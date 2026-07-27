# Hardware bring-up gate

This is the shortest controlled path from the built artifact to a manual driveway recording run. It does not authorize an unattended or loaded driveway run.

## 1. Build and flash the ESP32

```sh
chmod +x donkeycar/scripts/build-flash-esp32.sh
donkeycar/scripts/build-flash-esp32.sh /dev/ttyUSB0
```

Use the actual ESP32 programming port. The script builds `esp32_drive_coordinator` before uploading it.

## 2. Install the Raspberry Pi runtime

```sh
chmod +x donkeycar/scripts/install-pi.sh
./donkeycar/scripts/install-pi.sh
source donkeycar/.venv/bin/activate
```

Confirm the Pi user belongs to `dialout` and `video`. Reboot after changing groups.

## 3. Restrain the chassis

Lift both hoverboard wheels clear of the floor. Block the casters. Remove the trash can. Keep immediate access to battery disconnect and local disarm.

## 4. Run zero-motion preflight

```sh
source donkeycar/.venv/bin/activate
python donkeycar/scripts/preflight.py --config donkeycar/config/robot.yaml
```

A pass proves only that the Pi opened the serial port, negotiated protocol v1, found a non-dry-run drive image, selected drive mode, received a fault-free status response, and sent stop plus disarm before closing.

## 5. Verify wheel direction at minimum demand

Start the vehicle in manual mode:

```sh
cd donkeycar
python manage.py drive --config config/robot.yaml
```

Use the web controller at port 8887. Apply the smallest positive throttle possible. Both wheels must correspond to chassis-forward travel. Verify steering causes opposite differential changes. Stop immediately if either sign is wrong.

## 6. Prove every software stop

With wheels still lifted, separately verify:

1. Releasing throttle commands zero.
2. Stopping the Donkeycar process commands zero and disarms.
3. Unplugging Pi USB lets the 500 ms lease expire and disables demand.
4. Removing master feedback faults and disables demand.
5. Tilting beyond the configured limit disables demand.
6. Local disarm disables demand.
7. Reconnection begins with a new hello, drive-mode selection, and zero demand.

Do not proceed if any stop depends only on the dashboard display.

## 7. Record a low-speed manual tub

Lower the robot on flat ground first, without the trash can. Limit throttle to walking pace and keep a spotter at the battery disconnect. Record a short standard Donkeycar tub, inspect images and controls, then repeat on the driveway only after flat-ground steering and braking are predictable.

## 8. Train and run pilot mode

Train with Donkeycar tooling, copy the model into `donkeycar/models`, update YAML, and run:

```sh
python manage.py drive --config config/robot.yaml --model models/driveway.h5
```

Pilot mode remains gated by the ESP32 connection. A model is not a safety system. Keep manual intervention available for every early run.

## Optional service installation

After interactive validation succeeds:

```sh
chmod +x donkeycar/scripts/install-service.sh
donkeycar/scripts/install-service.sh
sudo systemctl start trashcan-donkeycar.service
journalctl -u trashcan-donkeycar.service -f
```

Do not enable automatic physical movement at boot. The service starts the software only, and the ESP32 still requires an explicit drive session and fresh leased commands.
