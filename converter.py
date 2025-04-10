import numpy as np
import cv2
import serial
import time
import struct
import threading
import os


# --- Настройки ---
SERIAL_PORT = 'COM5'
BAUD_RATE = 115200
SAMPLING_FREQUENCY = 64
RADIAL_SCALE = 1.0
CENTER_X = 0
CENTER_Y = 0
UPDATE_INTERVAL = 0.00104167


def cartesian_to_polar(img, center=None, sampling_frequency=360, radial_scale=1.0):

    height, width = img.shape[:2]

    if center is None:
        center = (width // 2, height // 2)

    max_radius = center[0]

    polar_img = np.zeros((max_radius * 2, sampling_frequency, img.shape[2]), dtype=img.dtype) if len(img.shape) == 3 else np.zeros((max_radius * 2, sampling_frequency), dtype=img.dtype)
    view_img = np.zeros((center[1] * 2, center[0] * 2, img.shape[2]), dtype=img.dtype)

    for angle in range(sampling_frequency):
        for i in range(2):
            theta = np.pi * angle / sampling_frequency

            for radius in range(max_radius):
                r = radius / radial_scale

                x = int(center[0] + r * np.cos(theta + np.pi * i))
                y = int(center[1] + r * np.sin(theta + np.pi * i))

                if 0 <= x < width and 0 <= y < height:
                    if i == 0:
                        k = 1
                    else:
                        k = -1
                    polar_img[(k * radius) + max_radius, angle] = img[y, x]
                    view_img[y, x] = img[y, x]
                else:
                    polar_img[radius, angle] = [0, 0, 0]
            cv2.imshow("View Image", view_img)
            cv2.imshow("Polar Image", polar_img)
            cv2.waitKey(1)
            cv2.destroyAllWindows()

    return polar_img


def convert(f):
    img = cv2.imread(f'img\\basic\\{f}')

    if img is None:
        print("Ошибка: Не удалось загрузить изображение. Убедитесь, что файл существует и доступен.")
    else:

        sampling_frequency = SAMPLING_FREQUENCY
        radial_scale = RADIAL_SCALE
        center = (img.shape[1] // 2, img.shape[0] // 2)

        polar_img = cartesian_to_polar(img, center=center, sampling_frequency=sampling_frequency, radial_scale=radial_scale)

        cv2.imwrite(f'img\\polar\\{f.split(".")[0]}_polar.png', polar_img)
        print(f"Полярное изображение сохранено как {f.split(".")[0]}_polar.png")

        cv2.imshow("Исходное изображение", img)
        cv2.imshow("Полярное изображение", polar_img)

        cv2.waitKey(1000)
        cv2.destroyAllWindows()


def send_to_esp(ser, polar_slice):
    if polar_slice is None or len(polar_slice.shape) < 2:
        print("Ошибка: Неверный формат среза полярного изображения.")
        return

    for pixel_data in polar_slice:
        if len(pixel_data) == 3:
            r, g, b = pixel_data

            packed_data = struct.pack('<BBB', int(r), int(g), int(b))
            ser.write(packed_data)
        elif len(pixel_data) == 1:
            gray = pixel_data[0]
            packed_data = struct.pack('<B', int(gray))
            ser.write(packed_data)
        else:
            print("Ошибка: Неподдерживаемый формат цвета.")
            return
    ser.flush()


def update_image(ser, img, center, sampling_frequency, radial_scale):
    polar_img = cartesian_to_polar(img, center=center, sampling_frequency=sampling_frequency, radial_scale=radial_scale)
    num_angles = polar_img.shape[1]
    current_angle_index = 0

    while True:
        try:
            polar_slice = polar_img[:, current_angle_index]
            send_to_esp(ser, polar_slice)
            current_angle_index = (current_angle_index + 1) % num_angles
            time.sleep(UPDATE_INTERVAL)
        except serial.SerialException as e:
            print(f"Ошибка последовательного порта: {e}")
            break
        except KeyboardInterrupt:
            print("Прерывание пользователем.  Завершение потока.")
            break
        except Exception as e:
            print(f"Произошла ошибка: {e}")
            break


def start_convert():
    for addr, dirs, files in os.walk("img\\basic\\"):
        for f in files:
            if f.endswith((".png", ".jpg", ".jpeg", ".bmp", ".gif")):
                convert(f)


if __name__ == '__main__':
    start_convert()
    img = cv2.imread('img\\basic\\test.png')
    if img is None:
        print("Ошибка: Не удалось загрузить изображение.  Убедитесь, что файл существует.")
        exit()

    # Расчет центра (если не задан вручную)
    if CENTER_X == 0 and CENTER_Y == 0:
        center = (img.shape[1] // 2, img.shape[0] // 2)
    else:
        center = (CENTER_X, CENTER_Y)

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
        print(f"Подключено к {SERIAL_PORT} со скоростью {BAUD_RATE} бод.")

        image_thread = threading.Thread(target=update_image, args=(ser, img, center, SAMPLING_FREQUENCY, RADIAL_SCALE), daemon=True)
        image_thread.start()

        while True:
            try:
                time.sleep(1)
            except KeyboardInterrupt:
                print("Получено прерывание.  Завершение работы.")
                break

    except serial.SerialException as e:
        print(f"Ошибка при открытии последовательного порта: {e}")
    except Exception as e:
        print(f"Произошла неожиданная ошибка: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Последовательный порт закрыт.")
        print("Завершение работы.")