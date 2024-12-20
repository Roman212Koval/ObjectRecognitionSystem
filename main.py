import os
import time
import cv2
import keyboard as kp
import numpy as np
from djitellopy import Tello
import flet as ft
import threading
import base64
from ultralytics import YOLO
import math
from PIL import Image as PILImage
from io import BytesIO


class DroneController:
    def __init__(self):
        self.drone = Tello()
        self.drone.connect()
        self.drone.streamon()
        self.is_flying = False
        self.tracking_active = False

    def get_video_frame(self):
        """Отримання кадру відео з дрона та корекція кольору"""
        frame = self.drone.get_frame_read().frame
        frame = cv2.resize(frame, (640, 640))
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        return frame


    def get_keyboard_input(self):
        """Обробка клавіатурних команд для управління дроном"""
        lr, fb, ud, yv = 0, 0, 0, 0
        speed = 80
        liftSpeed = 80
        moveSpeed = 85
        rotationSpeed = 100

        try:
            if kp.is_pressed("LEFT"):
                lr = -speed
            elif kp.is_pressed("RIGHT"):
                lr = speed

            if kp.is_pressed("UP"):
                fb = moveSpeed
            elif kp.is_pressed("DOWN"):
                fb = -moveSpeed

            if kp.is_pressed("w"):
                ud = liftSpeed
            elif kp.is_pressed("s"):
                ud = -liftSpeed

            if kp.is_pressed("d"):
                yv = rotationSpeed
            elif kp.is_pressed("a"):
                yv = -rotationSpeed

            if kp.is_pressed("q"):
                self.stop_flight()
            elif kp.is_pressed("e"):
                self.start_flight()

            if kp.is_pressed("z"):  # зробити знімок
                img = self.get_video_frame()
                filename = f"tellopy/Resources/Images/{time.time()}.jpg"
                cv2.imwrite(filename, img)
                time.sleep(0.3)
        except Exception as e:
            print(f"Помилка при зчитуванні команд: {e}")

        return [lr, fb, ud, yv]

    def start_flight(self):
        if not self.is_flying:
            self.drone.takeoff()
            time.sleep(1)  # час на взліт
            self.is_flying = True  # оновлюємо стан

    def stop_flight(self):
        if self.is_flying:
            self.drone.land()
            time.sleep(1)  # час на приземлення
            self.is_flying = False  # оновлюємо стан

    def set_tracking_state(self, state: bool):
        self.tracking_active = state

    def send_rc_control(self, commands):
        """Відправка команд на управління дроном"""
        if True:
            try:
                self.drone.send_rc_control(commands[0], commands[1], commands[2], commands[3])
            except Exception as e:
                # Логування помилки або інші дії
                print(f"Помилка при відправці команд на дрон: {e}")

    def take_screenshot(self):
        """Зробити скріншот з відеопотоку"""
        img = self.get_video_frame()
        filename = f"tellopy/Resources/Images/{time.time()}.jpg"
        cv2.imwrite(filename, img)
        time.sleep(0.3)


class Detector:
    def __init__(self, model_path):
        self.model = YOLO(model_path, task="detect")

    def detect_and_annotate(self, frame, selected_class=None):
        """Детекція об'єктів, накладення розмітки та визначення напрямку руху для дрона"""
        results = self.model.predict(source=frame)

        # Накладання результатів на кадр
        annotated_frame = frame.copy()
        height, width, _ = frame.shape

        # Визначення центральної області
        central_rect_width = int(width * 0.25)
        central_rect_height = int(height * 0.25)
        central_rect_x1 = int((width - central_rect_width) / 2)
        central_rect_y1 = int((height - central_rect_height) / 2) - 100
        central_rect_x2 = central_rect_x1 + central_rect_width
        central_rect_y2 = central_rect_y1 + central_rect_height

        # Малюємо центральну область
        cv2.rectangle(
            annotated_frame,
            (central_rect_x1, central_rect_y1),
            (central_rect_x2, central_rect_y2),
            (255, 0, 0),
            2,
        )

        # Центр центральної області
        central_rect_center_x = int((central_rect_x1 + central_rect_x2) / 2)
        central_rect_center_y = int((central_rect_y1 + central_rect_y2) / 2)

        # Масив для команд дрона
        drone_commands = [0, 0, 0, 0]  # [lr, fb, ud, yv]

        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls = self.model.names[int(box.cls[0])]
            conf = box.conf[0]

            # Перевіряємо, чи об'єкт відповідає вибраному класу
            if selected_class is None or selected_class == "All" or cls == selected_class:
                # Обчислюємо центр об'єкта
                obj_center_x = int((x1 + x2) / 2)
                obj_center_y = int((y1 + y2) / 2)

                # Обчислюємо розмір bounding box
                box_width = x2 - x1
                box_height = y2 - y1
                box_area = box_width * box_height

                # Встановлюємо порогові значення для відстані
                too_close_threshold = (width * height) * 0.18  # 18% від площі кадру
                too_far_threshold = (width * height) * 0.08    # 8% від площі кадру

                # Визначаємо стан об'єкта на основі його площі (відстань)
                if box_area >= too_close_threshold:
                    distance_status = "Close"
                    distance_color = (0, 165, 255)  # Помаранчевий

                    # Розрахунок перевищення площі
                    excess = box_area - too_close_threshold
                    max_excess = (width * height) - too_close_threshold
                    # Масштабуємо швидкість до межі, до -100
                    speed = -int((excess / max_excess) * 100)
                    speed = max(speed, -84)  # Обмежуємо мінімальну швидкість
                    drone_commands[1] = speed  # Летіти назад

                elif box_area <= too_far_threshold:
                    distance_status = "Far"
                    distance_color = (0, 255, 255)  # Жовтий

                    # Розрахунок недостачі площі
                    deficit = too_far_threshold - box_area
                    # Масштабуємо швидкість до межі, наприклад, до 100
                    speed = int((deficit / too_far_threshold) * 100)
                    speed = min(speed, 84)  # Обмежуємо максимальну швидкість
                    drone_commands[1] = speed  # Летіти вперед

                else:
                    distance_status = " "
                    distance_color = (0, 255, 0)  # Зелений
                    drone_commands[1] = 0  # Не рухатись

                # Визначаємо, чи об'єкт в центральній області
                if central_rect_x1 <= obj_center_x <= central_rect_x2 and central_rect_y1 <= obj_center_y <= central_rect_y2:
                    position_status = "In focus"
                else:
                    position_status = "Out of focus"

                    # Обчислюємо напрямок руху до центру
                    dx = central_rect_center_x - obj_center_x
                    dy = central_rect_center_y - obj_center_y

                    # Нормалізуємо вектор до довжини 50 пікселів
                    length = math.hypot(dx, dy)
                    if length > 0:
                        dx_norm = int(dx / length * 50)
                        dy_norm = int(dy / length * 50)
                    else:
                        dx_norm = 0
                        dy_norm = 0

                    # Повороти: швидкість змінюється в залежності від того, як далеко об'єкт від центру
                    turn_speed = int((abs(dx) / width) * 200)  # Чим далі від центру, тим швидше поворот
                    if dx > 0:
                        drone_commands[3] = -turn_speed  # Поворот вліво
                    elif dx < 0:
                        drone_commands[3] = turn_speed  # Поворот вправо

                    # Вгору / вниз: змінюємо швидкість в залежності від вертикального відхилення
                    vertical_speed = int((abs(dy) / height) * 120)  # Швидкість вертикального руху
                    if dy > 0:
                        drone_commands[2] = vertical_speed * 2  # Вгору
                    elif dy < 0:
                        drone_commands[2] = -vertical_speed  # Вниз

                    # Малюємо стрілку для напрямку руху
                    arrow_end = (obj_center_x + dx_norm, obj_center_y + dy_norm)
                    cv2.arrowedLine(
                        annotated_frame,
                        (obj_center_x, obj_center_y),
                        arrow_end,
                        distance_color,
                        2,
                        tipLength=0.5,
                    )

                # Малюємо рамку об'єкта
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), distance_color, 2)

                # Малюємо текст
                status_text = f"{cls} {conf:.2f} | {position_status} | {distance_status}"
                cv2.putText(
                    annotated_frame,
                    status_text,
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    distance_color,
                    2,
                )

                # Малюємо центр об'єкта
                cv2.circle(annotated_frame, (obj_center_x, obj_center_y), 5, distance_color, -1)

        return annotated_frame, drone_commands


class DroneApp:
    def __init__(self):
        self.detector = Detector("yolo11n_8_up2.engine")
        self.drone_controller = DroneController()
        self.cap = None
        self.video_cap = None
        self.camera_active = False
        self.threading_active = False
        self.tracking_active = False
        self.selected_class = None
        self.img_element = None
        self.page = None

    def encode_frame_to_base64(self, frame):
        """Перетворення кадру відео в Base64"""
        _, buffer = cv2.imencode('.jpg', frame)
        return base64.b64encode(buffer).decode('utf-8')

    def update_video_stream(self, page: ft.Page):
        """Оновлення відеопотоку на сторінці Flet з детекцією об'єктів"""
        while self.threading_active:
            if self.camera_active:
                # Отримуємо кадр відео
                frame = self.drone_controller.get_video_frame()

                # Детекція об'єктів та накладення розмітки
                annotated_frame, commands = self.detector.detect_and_annotate(frame, self.selected_class)
                self.display_frame(annotated_frame)
                if self.drone_controller.tracking_active:
                    self.drone_controller.send_rc_control(commands)

    def control_drone(self):
        """Контроль за дроном через клавіатурні команди"""
        while self.threading_active and not self.tracking_active:
            if self.camera_active:
                # Отримуємо команди управління
                commands = self.drone_controller.get_keyboard_input()

                # Застосовуємо команди до дрона
                self.drone_controller.send_rc_control(commands)

                time.sleep(0.03)  # невелика затримка для стабільної роботи

    def display_frame(self, frame):
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pil_img = PILImage.fromarray(frame_rgb)
        buffered = BytesIO()
        pil_img.save(buffered, format="JPEG")
        img_str = base64.b64encode(buffered.getvalue()).decode()
        self.img_element.src_base64 = img_str
        self.img_element.update()

    def capture_snapshot(self, _):
        """Збереження фото з відеопотоку"""
        if self.camera_active:
            frame = self.drone_controller.get_video_frame()
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            pil_img = PILImage.fromarray(frame_rgb)
            save_path = f"image/{time.time()}.jpg"
            pil_img.save(save_path, format="JPEG")
            buffered = BytesIO()
            pil_img.save(buffered, format="JPEG")
            img_str = base64.b64encode(buffered.getvalue()).decode()

            # Оновлення зображення в інтерфейсі
            self.img_element.src_base64 = img_str
            self.img_element.update()

            # Показуємо повідомлення
            snack_bar = ft.SnackBar(ft.Text(f"Фото збережено: {save_path}"))
            self.page.overlay.append(snack_bar)
            snack_bar.open = True
            self.page.update()

    def toggle_drone(self, _):
        """Активація/деактивація дрона"""
        if not self.camera_active:
            self.threading_active = True
            # Запускаємо контроль за дроном в окремому потоці
            threading.Thread(target=self.control_drone, daemon=True).start()

            # Запускаємо поток для оновлення відеопотоку
            threading.Thread(target=self.update_video_stream, args=(self.page,), daemon=True).start()
            # Спочатку активуємо відеопотік
            self.drone_controller.drone.streamon()
            time.sleep(0.3)
            self.camera_active = True

            # Відображаємо повідомлення
            snack_bar = ft.SnackBar(ft.Text("Дрон активовано"))
            self.page.overlay.append(snack_bar)
            snack_bar.open = True
            self.page.update()

        else:
            # Спочатку вимикаємо відеопотік
            self.drone_controller.drone.streamoff()

            self.camera_active = False
            self.threading_active = False

            # Відображаємо повідомлення
            snack_bar = ft.SnackBar(ft.Text("Дрон деактивовано"))
            self.page.overlay.append(snack_bar)
            snack_bar.open = True
            self.page.update()

    def toggle_flying(self, _):
        """Активація/деактивація польоту"""
        if self.drone_controller.drone.is_flying:
            self.drone_controller.stop_flight()
            snack_bar = ft.SnackBar(ft.Text("Дрон приземлено"))
            self.page.overlay.append(snack_bar)
            snack_bar.open = True
            self.page.update()
        else:
            self.drone_controller.start_flight()
            snack_bar = ft.SnackBar(ft.Text("Дрон в режимі польоту"))
            self.page.overlay.append(snack_bar)
            snack_bar.open = True
            self.page.update()

    def toggle_tracking(self, _):
        """Активація/деактивація трекінгу"""
        if self.drone_controller.drone.is_flying:
            if self.drone_controller.tracking_active:
                message_text = f"Трекінг деактивовано"
                self.drone_controller.set_tracking_state(False)
                threading.Thread(target=self.control_drone, daemon=True).start()
            else:
                message_text = f"Трекінг активовано для: {self.selected_class}"
                self.drone_controller.set_tracking_state(True)
        else:
            message_text = "Дрон не в стані польоту"
        self.tracking_active = self.drone_controller.tracking_active
        snack_bar = ft.SnackBar(ft.Text(message_text))
        self.page.overlay.append(snack_bar)
        snack_bar.open = True
        self.page.update()

    def main(self, page: ft.Page):
        """Головна функція для запуску Flet інтерфейсу"""
        self.page = page
        page.window_width = 640
        page.window_height = 640
        page.title = "DJI Tello Drone Control with Object Detection"
        page.vertical_alignment = ft.MainAxisAlignment.START

        # Додаємо контейнер для відео
        self.img_element = ft.Image(width=640, height=480)
        container = self.img_element

        page.add(container)

        # Отримуємо список класів з моделі
        class_names = list(self.detector.model.names.values())
        class_names.append('All')

        # Створюємо випадаючий список
        class_dropdown = ft.Dropdown(
            width=120,
            options=[ft.dropdown.Option(name) for name in class_names],
            on_change=self.on_class_change,
        )

        # Додаємо кнопки управління
        control_panel = ft.Row(
            [
                ft.ElevatedButton("Дрон", on_click=self.toggle_drone),
                ft.ElevatedButton("Знімок", on_click=self.capture_snapshot),
                ft.ElevatedButton("Взліт/Посадка", on_click=self.toggle_flying),
                class_dropdown,
            ],
            alignment="center",
        )

        self.file_picker = ft.FilePicker(on_result=self.open_media_file)
        page.overlay.append(self.file_picker)
        media_panel = ft.Row(
            [
                ft.ElevatedButton("Трекінг", on_click=self.toggle_tracking),
                ft.ElevatedButton(
                    "Відкрити", on_click=lambda _: self.file_picker.pick_files(allow_multiple=False)),

            ],
            alignment="center",
        )
        page.add(control_panel, media_panel)

    def on_class_change(self, event):
        self.selected_class = event.control.value
        print(f"Вибраний клас для відстеження: {self.selected_class}")

    def open_media_file(self, event: ft.FilePickerResultEvent):
        # Зупиняємо поточні потоки
        if not self.camera_active:

            # Закриваємо камеру, якщо вона відкрита
            if self.cap is not None and self.cap.isOpened():
                self.cap.release()
                self.cap = None

            # Закриваємо попередній відео-файл, якщо він відкритий
            if self.video_cap is not None and self.video_cap.isOpened():
                self.video_cap.release()
                self.video_cap = None

            if event.files and event.files[0].path:
                file_path = event.files[0].path
                file_ext = os.path.splitext(file_path)[1].lower()

                if file_ext in [".jpg", ".jpeg", ".png", ".bmp"]:
                    # Відкриваємо зображення з детекцією
                    pil_img = PILImage.open(file_path)
                    frame = cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)
                    # Виконання детекції
                    # Детекція об'єктів та накладення розмітки
                    annotated_frame = self.detector.detect_and_annotate(frame, self.selected_class)

                    self.display_frame(annotated_frame)

                elif file_ext in [".mp4", ".avi", ".mov", ".mkv"]:
                    # Встановлюємо FFmpeg опції для вимкнення багатопотоковості
                    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "threads;1"

                    # Відкриваємо відео
                    self.video_cap = cv2.VideoCapture(file_path, cv2.CAP_FFMPEG)
                    if not self.video_cap.isOpened():
                        self.page.add(ft.Text("Не вдалося відкрити відео-файл"))
                        return

                    self.video_fps = self.video_cap.get(cv2.CAP_PROP_FPS)
                    if self.video_fps == 0:
                        self.video_fps = 30  # Встановлюємо FPS за замовчуванням, якщо не вдалося отримати

                    # Запускаємо потік для відтворення відео
                    threading.Thread(target=self.update_video, daemon=True).start()
                else:
                    self.page.add(ft.Text("Непідтримуваний формат файлу"))
                    self.page.update()

    def update_video(self):
        while self.video_cap.isOpened() and not self.camera_active:
            ret, frame = self.video_cap.read()
            if not ret:
                break
            # Детекція об'єктів та накладення розмітки
            annotated_frame, _ = self.detector.detect_and_annotate(frame, self.selected_class)

            self.display_frame(annotated_frame)
            # Затримка для відповідності FPS відео
            # time.sleep(5 / self.video_fps)
        if self.video_cap is not None:
            self.video_cap.release()
            self.video_cap = None


# Запуск Flet
def run_app():
    app = DroneApp()
    ft.app(target=app.main)


run_app()
