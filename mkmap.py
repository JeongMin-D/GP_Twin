from PIL import Image
import numpy as np
import yaml

def load_jpg_map(file_path):
    image = Image.open(file_path)
    image = image.convert("L")  # 그레이스케일로 변환

    # 이미지를 numpy 배열로 변환
    map_data = np.array(image)  # 이미지를 numpy 배열로 변환
    map_data = 1 - (map_data / 255.0)  # 0과 1 사이의 값으로 정규화

    return map_data

def save_map_as_pgm(map_data, file_path):
    image = Image.fromarray((map_data * 255).astype('uint8'), mode='L')
    image.save(file_path)

def save_map_metadata_yaml(map_data, metadata_file_path, resolution, origin):
    metadata = {
        'image': 'map.pgm',
        'resolution': resolution,
        'origin': origin,
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196
    }

    with open(metadata_file_path, 'w') as file:
        yaml.dump(metadata, file)

if __name__ == '__main__':
    file_path = 'D:\\다운로드\\map.png'  # 사용할 jpg 파일 경로를 입력하세요
    map_data = load_jpg_map(file_path)

    pgm_file_path = 'D:\\다운로드\\map.pgm'  # 저장할 PGM 파일 경로 및 이름
    save_map_as_pgm(map_data, pgm_file_path)

    yaml_file_path = 'D:\\다운로드\\map.yaml'  # 저장할 YAML 파일 경로 및 이름
    resolution = 0.001  # 맵 해상도 (미터 단위)
    origin = [-10.0, -10.0, 0.0]  # 맵 원점 (미터 단위)
    save_map_metadata_yaml(map_data, yaml_file_path, resolution, origin)

    print(f"Map data saved as '{pgm_file_path}' and '{yaml_file_path}'")
