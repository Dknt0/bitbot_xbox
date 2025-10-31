Bitbot Joystick Frontend
===

Xbox Joystick frontend for [bitbot](https://bitbot.lmy.name).

# Requirements

```bash
sudo apt install xboxdrv joystick
```

Xbox linux driver [xpadneo](https://github.com/atar-axis/xpadneo):

```bash
git clone https://github.com/atar-axis/xpadneo.git
cd xpadneo
sudo ./install.sh
```

You may need to change button&axis mappings in `config/frontend.yaml`, depending on the version of driver and firmware.

TODO:

* [ ] Add udev rules for hidraw.
* [ ] Automatically find rumble device id.

