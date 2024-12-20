import os
import xml.etree.ElementTree as ET
import albumentations as A
import cv2


# Функція для зчитування bounding boxes з XML файлу
def read_bounding_boxes(xml_file_path):
    tree = ET.parse(xml_file_path)
    root = tree.getroot()
    bboxes = []
    class_labels = []
    for obj in root.findall('object'):
        bndbox = obj.find('bndbox')
        xmin = int(bndbox.find('xmin').text)
        ymin = int(bndbox.find('ymin').text)
        xmax = int(bndbox.find('xmax').text)
        ymax = int(bndbox.find('ymax').text)
        bboxes.append([xmin, ymin, xmax, ymax])
        class_labels.append(obj.find('name').text)
    return bboxes, class_labels


# Функція для запису bounding boxes до нового XML файлу
def write_bounding_boxes(original_xml_file_path, output_xml_file_path, bboxes):
    tree = ET.parse(original_xml_file_path)
    root = tree.getroot()
    for obj, bbox in zip(root.findall('object'), bboxes):
        bndbox = obj.find('bndbox')
        bndbox.find('xmin').text = str(bbox[0])
        bndbox.find('ymin').text = str(bbox[1])
        bndbox.find('xmax').text = str(bbox[2])
        bndbox.find('ymax').text = str(bbox[3])
    tree.write(output_xml_file_path)


# Функція для аугментації зображення з віддзеркаленням
def augment_image_and_bboxes(image_path, xml_file_path, output_folder):
    # Зчитування зображення
    image = cv2.imread(image_path)
    height, width, _ = image.shape

    # Зчитування bounding boxes
    bboxes, class_labels = read_bounding_boxes(xml_file_path)

    # Аугментація з віддзеркаленням
    transform = A.Compose([A.HorizontalFlip(p=1)],
                          bbox_params=A.BboxParams(format='pascal_voc', label_fields=['class_labels']))
    transformed = transform(image=image, bboxes=bboxes, class_labels=class_labels)
    transformed_image = transformed['image']
    transformed_bboxes = transformed['bboxes']

    # Збереження результатів
    base_filename = os.path.basename(image_path)
    filename_without_ext = os.path.splitext(base_filename)[0]

    output_image_path = os.path.join(output_folder, filename_without_ext + '_flipped.jpg')
    output_xml_path = os.path.join(output_folder, filename_without_ext + '_flipped.xml')

    cv2.imwrite(output_image_path, transformed_image)
    write_bounding_boxes(xml_file_path, output_xml_path, transformed_bboxes)


# Основна функція для обробки папки з датасетом
def process_dataset_folder(folder_path, output_folder):
    os.makedirs(output_folder, exist_ok=True)
    for filename in os.listdir(folder_path):
        if filename.endswith('.jpg'):
            image_path = os.path.join(folder_path, filename)
            xml_file_path = os.path.join(folder_path, os.path.splitext(filename)[0] + '.xml')
            if os.path.exists(xml_file_path):
                augment_image_and_bboxes(image_path, xml_file_path, output_folder)
                print(f'Augmented {image_path} and {xml_file_path}')


# Вкажіть шлях до вашої папки з датасетом та папки для збереження результатів
dataset_folder_path = 'C:/Users/koval/Desktop/test'
output_folder_path = 'C:/Users/koval/Desktop/Dataset3'
process_dataset_folder(dataset_folder_path, output_folder_path)
