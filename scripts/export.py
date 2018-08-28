import os
from datetime import datetime
from typing import List, Dict, Any

import numpy as np
import pandas as pd

from plotting import LatLngConverter
from math import isnan


# def point_to_geojson(lat: float, lng: float) -> Dict[str, Any]:
def point_to_geojson(lat, lng):
    """
    Converts a single x-y point to a GeoJSON dictionary object of coordinates
    :param lat: latitude of point
    :param lng: longitude of point
    :return: dictionary appropriate for conversion to JSON
    """
    feature = {
        'type': 'Feature',
        'geometry': {
            'type': 'Point',
            'coordinates': [lat, lng]
        }
    }
    return feature


def points_to_collection(x_vals, y_vals, convert, path_frame):
    """
    Converts many points to a JSON feature collection
    :param x_vals: list of x coordinates
    :param y_vals: list of y coordinates
    :param convert: converter object with the current data's origin and scale factor
    :param path: DataFrame of points along rover path
    :return: single FeatureCollection dictionary
    """

    features = {'type': 'FeatureCollection',
                'features': []}
    assert(len(x_vals) == len(y_vals))
    point_dicts = []
    point_frame = pd.DataFrame({'field.x': x_vals, 'field.y': y_vals})
    coord_frame = convert.data_to_latlng(point_frame)
    path_points = points_to_path(coord_frame, path_frame)
    for p in path_points.itertuples():
        point_dicts.append(point_to_geojson(p[1], p[2]))
    features['features'] = point_dicts

    return features


def points_to_path(coords, path):
    """Snaps the points to the closest place on the rover path"""

    dists = pd.DataFrame(index=np.arange(coords.shape[0]), columns=['path_ind',
                                                                    'dist'])

    for path_p in path.itertuples():

        for coord_p in coords.itertuples():
            newdist = np.linalg.norm(np.array((path_p[1], path_p[2]))
                                     - np.array((coord_p[1], coord_p[2])))
            if isnan(dists["dist"][coord_p[0]]) \
                    or dists["dist"][coord_p[0]] > newdist:
                dists["dist"][coord_p[0]] = newdist
                dists["path_ind"][coord_p[0]] = path_p[0]

    new_coords = pd.DataFrame(index=np.arange(coords.shape[0]),
                              columns=['field.latitude', 'field.longitude'])
    for i in range(dists.shape[0]):
        min_i = np.argmin(dists["path_ind"])[0]
        # Change to dists[dists = np.amin(dists)] eventually
        a = path["field.latitude"][dists["path_ind"][min_i]]
        new_coords["field.latitude"][i] = a
        b = path["field.longitude"][dists["path_ind"][min_i]]
        new_coords["field.longitude"][i] = b

        # Set path_index to an absurdly high value
        dists["path_ind"][min_i] = 99999999

    return coords


def read_data(file):
    data = np.genfromtxt(file, dtype=float, delimiter=',')
    return np.flipud(data)


def read_folder(dir, date_fmt):
    """
    Reads a folder of CSV files into an array. File names must contain date information.
    :param dir: directory containing all CSV files
    :param date_fmt: file names described in Python datetime format string syntax
    :return: 3D numpy array, with the third dimension describing time.
    """
    data = None
    files = os.listdir(dir)
    remove_index = []
    for i in range(len(files)):
        if files[i][-4:] != '.csv':
            remove_index.append(i)
    files = np.delete(files, remove_index)

    dates = [datetime.strptime(f, date_fmt) for f in files if f[-4:] == '.csv']
    sorted_files = [x for _, x in sorted(zip(dates, files))]

    for f in sorted_files:
        p = os.path.join(dir, f)
        if f.endswith('.csv'):
            if data is None:
                data = read_data(p)
            else:
                data = np.dstack((data, read_data(p)))
    return data


def read_path_file(path):

    main_frame = pd.read_csv(path)
    gps_points = main_frame.get(["field.latitude", "field.longitude",
                                 "field.altitude"])
    return gps_points
