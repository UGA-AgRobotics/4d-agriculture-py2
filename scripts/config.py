import configparser
import argparse
import os


# def create_config_global(file: os.path.abspath):
def create_config_global(file):
    config = configparser.ConfigParser(interpolation=None)
    config['GPS'] = {'origin_lat': '31.473566',
                     'origin_lng': '-83.529994',
                     'size': '0.2'}
    config['Filter'] = {'min_value': '2',
                        'max_value': '2'}
    config['Files'] = {'filename': r'height_%m%d%y.csv'}
    config['Path'] = {'DataPath': r"/home/wbyrnes3/gtri/src/agriculture-4d/data"}
    config['Points'] = {'MaxPoints': '10'}
    with open(file, 'w') as f:
        config.write(f)

    return config


# def create_config_yearly(file: os.path.abspath):
def create_config_yearly(file):
    config = configparser.ConfigParser(interpolation=None)
    config['GPS'] = {'origin_lat': '31.473566',
                     'origin_lng': '-83.529994',
                     'size': '0.2'}
    config['Filter'] = {'min_value': '1',
                        'max_value': '2'}
    config['Files'] = {'filename': r'height_%m%d%y.csv'}
    config['Path'] = {'DataPath': ""}
    config['Points'] = {'MaxPoints': '10'}
    with open(file, 'w') as f:
        config.write(f)

    return config


# def verify_config(file: os.path.abspath):
def verify_config(file):
    try:
        config = configparser.ConfigParser(interpolation=None)
        config.read(file)
        temp = config['GPS']['origin_lat']
        temp = config['GPS']['origin_lng']
        temp = config['GPS']['size']
        temp = config['Filter']['min_value']
        temp = config['Filter']['max_value']
        temp = config['Files']['filename']
        temp = config['Path']['DataPath']
        temp = config['Points']['MaxPoints']
    except KeyError:
        create_config_global(file)


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Create yearly config')
    parser.add_argument('path', type=str, default='config.ini',
                        help='Path to config file')
    path_ext = parser.parse_args().path

    global_parser = configparser.ConfigParser(interpolation=None)
    global_parser.read('config.ini')
    path_base = global_parser['Path']['DataPath']

    p = os.path.abspath(path_base + path_ext + "\config.ini")

    if not os.path.exists(p):
        create_config_yearly(p)
