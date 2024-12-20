import xml.etree.ElementTree as ET
import os
import shutil

classes = {
    'apple': 0,
    'bike': 1,
    'bottle': 2,
    'box': 3,
    'car': 4,
    'cup': 5,
    'laptop': 6,
    'person': 7,

    # Додайте інші класи за потреби
}


def convert_coordinates(img_width, img_height, box):
    x_min, y_min, x_max, y_max = box
    abs_x = (x_min + x_max) / 2.0
    abs_y = (y_min + y_max) / 2.0
    abs_width = x_max - x_min
    abs_height = y_max - y_min

    abs_x /= img_width
    abs_y /= img_height
    abs_width /= img_width
    abs_height /= img_height

    return abs_x, abs_y, abs_width, abs_height


xml_folder = 'C:/Users/koval/Desktop/test'
output_folder = 'C:/Users/koval/Desktop/Dataset_Yol'
image_destination_folder = output_folder

# Переконайтеся, що вихідна папка існує
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

for xml_file in os.listdir(xml_folder):
    if xml_file.endswith('.xml'):
        xml_path = os.path.join(xml_folder, xml_file)
        tree = ET.parse(xml_path)
        root = tree.getroot()

        # Оновлення назви файлу зображення
        base_name = os.path.splitext(xml_file)[0]
        img_name = base_name + '.jpg'

        filename_element = root.find('filename')
        if filename_element is not None:
            filename_element.text = img_name

        path_element = root.find('path')
        if path_element is not None:
            path_element.text = img_name
        else:
            path_element = ET.SubElement(root, 'path')
            path_element.text = img_name

        # Збереження оновленого XML файлу
        tree.write(xml_path, encoding='utf-8', xml_declaration=True)

        img_width = int(root.find('size/width').text)
        img_height = int(root.find('size/height').text)

        relevant_objects = []
        for obj in root.findall('object'):
            class_name = obj.find('name').text
            if class_name in classes:
                class_idx = classes[class_name]
                box = [
                    float(obj.find('bndbox/xmin').text),
                    float(obj.find('bndbox/ymin').text),
                    float(obj.find('bndbox/xmax').text),
                    float(obj.find('bndbox/ymax').text)
                ]
                relevant_objects.append((class_idx, box))

        if relevant_objects:
            img_source = os.path.join(xml_folder, img_name)
            img_destination = os.path.join(image_destination_folder, img_name)

            # Перевірка наявності файлу перед копіюванням
            if os.path.exists(img_source):
                shutil.copyfile(img_source, img_destination)
                txt_file = os.path.splitext(xml_file)[0] + '.txt'
                with open(os.path.join(output_folder, txt_file), 'w') as f:
                    for class_idx, box in relevant_objects:
                        x_center, y_center, width, height = convert_coordinates(img_width, img_height, box)
                        line = f"{class_idx} {x_center} {y_center} {width} {height}\n"
                        f.write(line)
            else:
                print(f"Зображення {img_source} не знайдено, пропускаємо цей файл.")

print("Обробка завершена.")