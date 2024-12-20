from PIL import Image, ImageEnhance
import xml.etree.ElementTree as ET
import os
import shutil

input_folder = 'C:/Users/koval/Desktop/test'
output_folder = 'C:/Users/koval/Desktop/Dataset3'

if not os.path.exists(output_folder):
    os.makedirs(output_folder)

for filename in os.listdir(input_folder):
    if filename.endswith((".png", ".jpg", ".jpeg")):
        img_path = os.path.join(input_folder, filename)
        img = Image.open(img_path)

        # Аугментація зображення
        enhancer = ImageEnhance.Brightness(img)
        img = enhancer.enhance(1.05)  # Збільшити яскравість на 5%

        enhancer = ImageEnhance.Contrast(img)
        img = enhancer.enhance(1.2)  # Збільшити контраст на 20%

        enhancer = ImageEnhance.Color(img)
        img = enhancer.enhance(1.1)  # Збільшити насиченість на 10%

        # Поворот зображення на 90 градусів
        img = img.rotate(90, expand=True)

        # Зберегти змінене зображення з новим суфіксом
        new_filename = os.path.splitext(filename)[0] + "_18" + os.path.splitext(filename)[1]
        img.save(os.path.join(output_folder, new_filename))

        # Обробка XML розмітки для цього зображення
        xml_filename = os.path.splitext(filename)[0] + ".xml"
        xml_path = os.path.join(input_folder, xml_filename)

        if os.path.exists(xml_path):
            tree = ET.parse(xml_path)
            root = tree.getroot()

            # Оновлення розмітки для аугментованого зображення
            size = root.find("size")
            img_width, img_height = img.size  # Нові розміри після повороту
            size.find("width").text = str(img_width)
            size.find("height").text = str(img_height)

            for obj in root.findall("object"):
                bndbox = obj.find("bndbox")
                x_min = int(float(bndbox.find("xmin").text))
                y_min = int(float(bndbox.find("ymin").text))
                x_max = int(float(bndbox.find("xmax").text))
                y_max = int(float(bndbox.find("ymax").text))

                # Перетворення координат після повороту на 90 градусів
                new_x_min = y_min
                new_y_min = img_width - x_max
                new_x_max = y_max
                new_y_max = img_width - x_min

                # Оновлення координат у розмітці
                bndbox.find("xmin").text = str(new_x_min)
                bndbox.find("ymin").text = str(new_y_min)
                bndbox.find("xmax").text = str(new_x_max)
                bndbox.find("ymax").text = str(new_y_max)

            # Зберегти нову XML розмітку з таким самим суфіксом
            new_xml_filename = os.path.splitext(new_filename)[0] + ".xml"
            tree.write(os.path.join(output_folder, new_xml_filename), encoding="utf-8", xml_declaration=True)

print("Аугментація завершена.")
