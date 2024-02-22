.PHONY: clean
clean:
	rm -rf .pio

.PHONY: teensy-firmware
teensy-firmware: clean
	pio run -e teensy40

.PHONY: teensy-upload
teensy-upload: clean
	pio run -e teensy40 --target upload --upload-port /dev/ttyACM0

.PHONY: pico-firmware
pico-firmware: clean
	pio run -e pico

.PHONY: pico-upload
pico-upload: clean
	pio run -e pico --target upload

.PHONY: feather-firmware
feather-firmware: clean
	pio run -e feather

.PHONY: feather-upload
feather-upload: clean
	pio run -e feather --target upload

.PHONY: monitor
monitor:
	pio device monitor

.PHONY: udev-rules
udev-rules:
	curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
