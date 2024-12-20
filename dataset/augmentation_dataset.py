import cv2
import os


def process_image(image_path, annotation_path, output_image_path, output_annotation_path, rotation_angle):
    image = cv2.imread(image_path)
    height, width = image.shape[:2]

    rotation_matrix = cv2.getRotationMatrix2D((width / 2, height / 2), rotation_angle, 1)
    rotated_image = cv2.warpAffine(image, rotation_matrix, (width, height))

    with open(annotation_path, 'r') as file:
        annotations = file.readlines()

    new_annotations = []
    for annotation in annotations:
        class_label, x, y, w, h = map(float, annotation.strip().split(' ')[0:5])
        x_center = x * width
        y_center = y * height
        box_width = w * width
        box_height = h * height

        new_x_center = rotation_matrix[0, 0] * x_center + rotation_matrix[0, 1] * y_center + rotation_matrix[0, 2]
        new_y_center = rotation_matrix[1, 0] * x_center + rotation_matrix[1, 1] * y_center + rotation_matrix[1, 2]
        new_box_width = box_width * rotation_matrix[0, 0]
        new_box_height = box_height * rotation_matrix[1, 1]

        new_x = new_x_center / width
        new_y = new_y_center / height
        new_w = new_box_width / width
        new_h = new_box_height / height

        new_annotation = f"{int(class_label)} {new_x} {new_y} {new_w} {new_h}\n"
        new_annotations.append(new_annotation)

    with open(output_annotation_path, 'w') as file:
        file.writelines(new_annotations)

    cv2.imwrite(output_image_path, rotated_image)


input_images_folder = 'C:/Users/koval/Desktop/Datasetro2'  # ---------------------------------------------------------------------
output_folder = 'C:/Users/koval/Desktop/Datasetro'

for filename in os.listdir(input_images_folder):
    if filename.endswith((".jpg", ".png")):
        image_path = os.path.join(input_images_folder, filename)
        annotation_path = os.path.join(input_images_folder, f"{os.path.splitext(filename)[0]}.txt")

        for i, rotation_angle in enumerate([-15, 15], start=1):
            output_image_path = os.path.join(output_folder, f"{os.path.splitext(filename)[0]}_{i}.jpg")
            output_annotation_path = os.path.join(output_folder, f"{os.path.splitext(filename)[0]}_{i}.txt")

            process_image(image_path, annotation_path, output_image_path, output_annotation_path, rotation_angle)
