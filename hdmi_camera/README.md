# HDMI Camera

Цей каталог містить скрипт **`main.py`** для перегляду зображення з камери OV5647 на HDMI екрані.

## Запуск

1. Встановіть необхідні пакети (для Raspberry Pi OS):
   ```bash
   sudo apt update
   sudo apt install -y python3-picamera2 python3-opencv --no-install-recommends
   ```
2. Перейдіть у каталог та запустіть скрипт. Можна вказати рядок підключення до
   MAVLink (`--mavlink`) та увімкнути цифрову стабілізацію (`--stabilize`).
   Стабілізація застосовується до зазумленої області у режимі "картинка в картинці":
   ```bash
   cd /path/to/hdmi_camera
   python3 main.py --mavlink udp:127.0.0.1:14550 --stabilize
   ```
   Після запуску зʼявиться превʼю з текстовим OSD, що показує стан зʼєднання та
   реле. Завершити роботу можна за допомогою `Ctrl+C`.

## Створення сервісу systemd

Щоб камера стартувала автоматично при завантаженні системи, створіть сервіс systemd.

1. Створіть файл `/etc/systemd/system/hdmi_camera.service` з наступним вмістом:
   ```ini
   [Unit]
   Description=HDMI Camera preview
   After=network.target

   [Service]
   ExecStart=/usr/bin/python3 /path/to/hdmi_camera/main.py
   WorkingDirectory=/path/to/hdmi_camera
   Restart=always
   User=pi

   [Install]
   WantedBy=multi-user.target
   ```
   Замініть `/path/to/hdmi_camera` на фактичний шлях до каталогу.
2. Активуйте сервіс:
   ```bash
   sudo systemctl daemon-reload
   sudo systemctl enable hdmi_camera.service
   ```
