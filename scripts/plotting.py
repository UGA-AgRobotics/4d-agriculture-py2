import configparser
from datetime import datetime
from math import cos
from skimage import filters
from skimage import measure
from math import radians
from scipy.interpolate import splprep, splev

import numpy as np
import pandas as pd
import scipy.ndimage as img

""" Tools to manipulate and analyze data """


def canopy_cover(data, radius):
    """Computes leaf area index: ratio of leaf to ground for a certain area.

    Performs a convolve with an arbitrary radius to calculate how many
    nonzero values are present within a
    :param data: 2D or 3D numpy array of height data
    :param radius: radius of the region (in number of 0.1 m squares)
    :return: leaf area index computed for each point
    """
    c = np.zeros_like(data, int)
    kernel = np.ones((radius * 2 + 1, radius * 2 + 1))
    for x in range(data.shape[2]):
        d = data[:, :, x]
        d[d > 0] = 1
        conv = img.filters.convolve(d, kernel)
        c[:, :, x] = conv

    return c


def create_path_splines(points, data_shape, converter):
    """Create managable path splines from the thousands of datapoints"""

    # Unpack rows and columns from data shape
    rows, cols, _ = data_shape

    # Convert the gps information and store it into a new DataFrame
    path_pts = []
    for p in points.itertuples():
        # This gives us the data coordinates from the gps location
        data_y = int(converter.lat_to_y(p[1]))
        data_x = int(converter.lng_to_x(p[2]))

        path_pts.append([data_x, data_y])

    np.transpose(path_pts)

    # Remove any duplicate points from the path,
    # keeping the original array order
    path_vals, ind = np.unique(path_pts, axis=0, return_index=True)
    ind = sorted(ind)
    path_pts = [path_pts[i] for i in ind]

    # Create a spline from the remaining path points
    # noinspection PyTupleAssignmentBalance
    tck, u = splprep(np.transpose(path_pts), u=None, s=0.0)
    return tck


def create_stress_map(height, canopy, rad, threshold):
    """Create map showing frequently stressed areas of plot.

    Sums the stress masks for each individual date to determine which
    areas are frequently coming up stressed.
    :return - The 2D map of stressed locations
    """

    # Deprecated create_suggestion_mask_v1 code
    ###########################################################################
    # #    Suggestion mask data (Normalized Height + Normalized Canopy)
    # normh = np.divide(height, np.amax(height, axis=(0, 1))) * 50
    # normc = np.divide(canopy, np.amax(canopy, axis=(0, 1))) * 50
    #
    # #    Process data to create stress mask for each snapshot
    # comb_data = np.add(normh, normc)
    # stress_dates = create_suggestion_mask_v1(comb_data, rad, threshold)
    ###########################################################################

    # Create the suggestion mask for each snapshot
    stress_dates = create_suggestion_mask_v2(height, canopy, rad, threshold)

    # Create stress map based on all data up to current date
    stress_map = np.zeros_like(stress_dates)
    for i in range(stress_dates.shape[2]):
        stress_map[:, :, i] = np.sum(stress_dates[:, :, 0:(i+1)], axis=2)
        stress_map[:, :, i] = np.divide(stress_map[:, :, i], (i+1))

    return stress_map


def create_suggestion_mask_v1(d, rad, threshold):
    """Keep this here for a little while, then delete it. Suggestion mask v1

    Uses statistical methods to determine outliers below the general
    population of the data, and uses image processing techniques to discount
    the edges of the plot from skewing the results.
    :param d: The input data to create the mask
    :param rad: The radius to average and desample the data
    :param threshold: The percentile above which points will be filtered
    :return: The mask from which suggested points are chosen
    """

    # Create a new copy of the data to work on
    data = np.copy(d)

    # filter out data less than zero
    data[data < 0] = 0

    # Calculates each point as sum of nearby values within radius r
    c = np.zeros_like(data, float)
    kernel = np.ones((rad * 2 + 1, rad * 2 + 1))
    for x in range(data.shape[2]):
        conv = img.filters.convolve(data[:, :, x], kernel)
        c[:, :, x] = conv

    # Downsample array into pixels with same size as convolve
    c = c[::rad * 2 + 1, ::rad * 2 + 1, :]

    fullmask = np.zeros_like(d)
    for i in range(c.shape[2]):
        # Extract the ith layer of data
        mask = c[:, :, i]

        # Use image processing morphology to smooth out data
        mask = img.grey_closing(mask, structure=np.ones((3, 3)))

        # Use Sobel edge detection to decrease weight of edges
        gx = img.sobel(mask, axis=0)
        gy = img.sobel(mask, axis=1)
        grad = np.hypot(gx, gy)
        grad = (np.divide(grad, np.amax(grad))) * 100
        mask = (np.divide(mask, np.amax(mask))) * 100
        mask -= grad

        # Calculate the threshold percentile, ignoring zeros
        mask[mask <= 0] = np.nan
        percent = np.nanpercentile(mask, threshold)
        mask = np.nan_to_num(mask)

        # Filter out data and create mask
        mask[mask > percent] = 0
        mask[mask > 0] = 1

        # Perform binary opening to remove small regions
        mask = img.binary_opening(mask)

        # Rescale mask to fit data size
        scale = np.divide(fullmask[:, :, 0].shape, mask.shape)
        fullmask[:, :, i] = img.zoom(mask, scale, order=0)

    return fullmask


def create_suggestion_mask_v2(height, canopy, rad=4, threshold=(20, 40)):

    # Copy the data
    height_data = np.copy(height)

    # Silence isolated points (low canopy)
    height_data[canopy < 5] = 0

    # Downscale dataset to 0.5m squares, taking the max within each
    height_data = downscale_max(height_data, rad)

    # Place points into stress levels
    stress_data = np.zeros_like(height_data)
    for x in range(stress_data.shape[2]):
        stress_layer = stress_data[:, :, x]
        height_layer = height_data[:, :, x]

        high_med_stress = np.percentile(height_layer[np.nonzero(height_layer)],
                                        threshold[0])
        med_low_stress = np.percentile(height_layer[np.nonzero(height_layer)],
                                       threshold[1])

        stress_layer[height_layer >= med_low_stress] = 0.01  # Low
        height_layer[stress_layer > 0] = 0  # silence low points

        stress_layer[height_layer >= high_med_stress] = 0.5  # Medium
        height_layer[stress_layer > 0] = 0  # silence med points

        stress_layer[0 < height_layer] = 0.99  # High

        stress_data[:, :, x] = stress_layer

    stress_data = rescale_like(stress_data, height)

    return stress_data


def define_regions(data, rad):
    """Identify regions of high stress areas and """

    region_map = np.copy(data)
    region_map = region_map[::rad * 2 + 1, ::rad * 2 + 1]

    val = filters.threshold_otsu(region_map)
    mask = region_map > val

    mask = img.binary_opening(mask, iterations=2)

    scale = np.divide(data.shape, mask.shape)
    mask = img.zoom(mask, scale, order=0)

    labels = measure.label(mask, background=0)
    regions = img.find_objects(labels)

    small_regions = []
    for i in range(len(regions)):
        if np.nonzero(labels == i + 1)[0].size <= 500:
            labels[regions[i]] = 0
            small_regions.append(i)

    for i in small_regions[::-1]:
        del regions[i]

    return StressMapWrapper(labels, regions)


def downscale_avg(data, radius):

    # Calculates each point as sum of nearby values within radius r
    diam = 2 * radius + 1

    kernel = np.ones((diam, diam))
    fullmap = np.zeros_like(data, float)
    for x in range(data.shape[2]):
        conv = img.filters.convolve(data[:, :, x], kernel)
        fullmap[:, :, x] = conv

    # Downsample array into pixels with same size as convolve
    fullmap = fullmap[::diam, ::diam, :]

    return fullmap


def downscale_max(data, radius):

    # Turn radius into diameter centered at original point
    diam = 2 * radius + 1

    fullmap = np.zeros_like(data[::diam, ::diam, :], float)
    for x in range(data.shape[2]):

        layer = np.zeros_like(data[::diam, ::diam, 0])
        for r in range((int(data.shape[0] / diam)) - 1):
            for c in range((int(data.shape[1] / diam)) - 1):
                selection = data[(r*diam):(r*diam + diam),
                                 (c*diam):(c*diam + diam), x]
                max_val = np.amax(selection)
                layer[r, c] = max_val

        fullmap[:, :, x] = layer

    return fullmap


def evaluate_path_spline(spline, num_points):

    u_vals = np.linspace(0, 1, num_points)
    pts = splev(u_vals, spline)

    pts = pd.DataFrame(np.transpose(pts))

    return pts


def filter_outliers(data, rmin = 0.2, rmax = 1.0):
    """Sets outliers outside user defined range to zero.

    Calculates the average and standard deviation of the dataset. Filters out
    all data below rmin * std and avg + rmax * std.
    :param data: 2D numpy array of data to filter
    :param rmin: data within (r * standard deviation) of zero is set to zero
    :param rmax: data above (average + r * standard deviation) is set to zero
    :return: filtered data
    """

    for x in range(data.shape[2]):
        d = np.nan_to_num(data[:, :, x])

        # Silence negative values
        d[d < 0] = 0

        # Calculate average and std
        avg = np.average(d[np.nonzero(d)])
        std = np.std(d[np.nonzero(d)])

        d[np.absolute(d) < avg - (rmin * std)] = 0  # filter points below avg
        d[np.absolute(d) > avg + (rmax * std)] = 0  # filter points above avg
        data[:, :, x] = d
    return data


def rescale_like(data, like):

    # Rescale mask to fit data size
    # scale = np.divide(like[:, :, 0].shape, data[:, :, 0].shape)
    scale = np.true_divide(like[:, :, 0].shape, data[:, :, 0].shape)

    fullmap = np.zeros_like(like, float)
    for x in range(data.shape[2]):
        fullmap[:, :, x] = img.zoom(data[:, :, x], scale, order=0)

    return fullmap


class DataSet4D:
    """This class contains all of the data for a particular mode.

    This class is responsible for handling the datasets for each different
    mode or filter. It has behaviors to change the date, perform statistics,
    and manipulate the data to some extent."""

    def __init__(self, data, dates):

        # Expand dimensions if necessary
        if len(data.shape) == 2:
            data = np.expand_dims(data, axis=2)

        self.data = np.nan_to_num(data)
        self.dates = sorted(dates)
        self.filter = filter

        date_length = self.dates.__len__()
        data_length = self.data.shape[2]

        # Duplicate last data element to match date length
        while data_length < date_length:
            self.data = np.concatenate((self.data,
                                        np.expand_dims(data[:, :, -1], axis=2)),
                                       axis=2)
            data_length += 1

        self._n_samples = self.data.shape[2]
        self._active_sample = 0
        self._active_data = self.data[:, :, self._active_sample]

        self.max_val = 0
        self.min_val = 0

        self.average = 0
        self.std_dev = 0
        self.pct_coverage = 0

        self.refresh_statistics()

    def get_data(self):
        """Get the 2D array of the map at the current date."""

        data = self._active_data
        return data

    def get_date(self):
        """Return the current date as a datetime object."""

        return self.dates[self._active_sample]

    def get_dates(self):
        """Return the backing array of datetime objects."""

        return self.dates

    def get_date_ind(self):
        """Return the index for the current date."""

        return self._active_sample

    def set_dates(self, dataset):
        """Copy the date object from another dataset."""

        self._n_samples = dataset.data.shape[2]
        self.dates = dataset.get_dates()

    def derivative(self):
        """Take the derivative of the dataset over time."""

        # If there is only one data set, return unchanged
        if self.data.shape[2] == 1:
            return self.data

        derivatives = np.empty([self.data.shape[0], self.data.shape[1],
                                self._n_samples - 1])
        diff = self.data[:, :, 1::] - self.data[:, :, 0:-1]
        for i in range(len(self.dates) - 1):
            date_interval = self.dates[i + 1] - self.dates[i]

            derivatives[:, :, i] = np.divide(diff[:, :, i], date_interval.days)

        return derivatives

    def next_data(self):
        """Advance the active data to the next date, if possible."""

        if self._active_sample >= self._n_samples - 1:
            self._active_sample = 0
        else:
            self._active_sample += 1
        self._active_data = self.data[:, :, self._active_sample]
        self.refresh_statistics()

    def prev_data(self):
        """Reqind the active data to the previous date, if possible."""

        if self._active_sample == 0:
            self._active_sample = self._n_samples - 1
        else:
            self._active_sample -= 1
        self._active_data = self.data[:, :, self._active_sample]
        self.refresh_statistics()

    def refresh_statistics(self):
        """Recalculate statistics for this dataset."""

        self.max_val = np.max(self._active_data)
        self.min_val = np.min(self._active_data)

        self.average = np.average(self._active_data[np.nonzero(
                                  self._active_data)])
        self.std_dev = np.std(self._active_data[np.nonzero(self._active_data)])
        self.pct_coverage = np.count_nonzero(self._active_data)\
            / (self._active_data.shape[0] * self._active_data.shape[1]) * 100

    def reset_date(self):
        """Reset the data to the first date in the dataset."""

        self._active_sample = 0
        self._active_data = self.data[:, :, 0]


class LatLngConverter:
    """This utility class converts x and y positions to coordinates."""

    def __init__(self, config):
        self.lng0 = float(config['GPS']['origin_lng'])
        self.lat0 = float(config['GPS']['origin_lat'])
        self.origin = (self.lat0, self.lng0)
        self.diam = float(config['GPS']['size'])

    def data_to_latlng(self, points):
        latlng = pd.DataFrame.copy(points)
        latlng.columns = ["field.latitude", "field.longitude"]
        for pt in points.itertuples():
            latlng["field.latitude"][pt[0]] = self.y_to_lat(pt[2])
            latlng["field.longitude"][pt[0]] = self.x_to_lng(pt[1])

        return latlng

    def lat_to_y(self, lat):

        # Inverse of above function
        d_lat = self.lat0 - lat
        m = d_lat * 111111
        y = m / self.diam

        return y

    def lng_to_x(self, lng):

        # Inverse of above function
        d_lng = self.lng0 - lng
        m = d_lng * (111111 * cos(radians(self.lat0)))
        x = m / self.diam

        return x

    def x_to_lng(self, x):
        # Convert data point to distance in meters
        m = x * self.diam

        # Convert data point to longitude with shortcut:
        # 1 deg lng = 111111 * cos(lat) * m
        d_lng = m / (111111 * cos(radians(self.lat0)))

        # Determine new longitude from base longitude
        return self.lng0 + d_lng

    def y_to_lat(self, y):
        # Convert data point to distance in meters
        m = y * self.diam

        # Convert data point to latitude with shortcut:
        # 1 deg lng = 111111
        d_lng = m / 111111

        # Determine new longitude from base longitude
        return self.lat0 + d_lng


class StressMapWrapper:
    def __init__(self, stress_map, regions):
        self.map = stress_map
        self.regions = regions


if __name__ == '__main__':
    '''Simple script to convert the lat/lng of a point relative to a given 
    anchor'''

    config = configparser.ConfigParser()
    config['GPS'] = {'origin_lat': '31.52036604680005',
                     'origin_lng': '-83.54861912284196',
                     'size': '0.2'}
    convert = LatLngConverter(config)

    x = -296
    y = -601

    print("lat: " + str(convert.y_to_lat(y)))
    print("lng: " + str(convert.x_to_lng(x)))
