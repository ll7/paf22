#!/usr/bin/python

from __future__ import print_function, absolute_import, division
from collections import namedtuple

# -----------------------------------------------------------------------------
# Definitions
# -----------------------------------------------------------------------------

# a label and all meta information
Label = namedtuple('Label', [

    'name',  # The identifier of this label, e.g. 'car', 'person', ...
    # We use them to uniquely name a class

    'id',  # An integer ID that is associated with this label.
    # The IDs are used to represent the label in ground
    # truth images
    # An ID of -1 means that this label does not have an ID
    # and thus is ignored when creating ground truth images.
    # Do not modify these IDs, since exactly these IDs are
    # expected by the evaluation server.

    'category',  # The name of the category that this label belongs to

    'categoryId',  # The ID of this category. Used to create ground truth
    # images on category level.

    'hasInstances',  # Whether this label distinguishes between single
    # instances or not

    'ignoreInEval',  # Whether pixels having this class as ground truth label
    # are ignored during evaluations or not

    'color',  # The color of this label
])

# -----------------------------------------------------------------------------
# A list of all labels
# -----------------------------------------------------------------------------

labels = [
    #     name   id   category  catId  hasInstances  ignoreInEval  color
    Label('none', 0, 'none', 0, False, True, (0, 0, 0)),
    Label('road', 1, 'flat', 1, True, False, (128, 64, 128)),
    Label('sidewalk', 2, 'flat', 1, True, True, (244, 35, 232)),
    Label('building', 3, 'construction', 2, True, True, (70, 70, 70)),
    Label('wall', 4, 'construction', 2, True, True, (102, 102, 156)),
    Label('fence', 5, 'construction', 2, True, True, (100, 40, 40)),
    Label('pole', 6, 'object', 3, True, False, (153, 153, 153)),
    Label('traffic light', 7, 'object', 3, True, False, (250, 170, 30)),
    Label('traffic sign', 8, 'object', 3, True, False, (220, 220, 0)),
    Label('vegetation', 9, 'nature', 4, False, True, (107, 142, 35)),
    Label('terrain', 10, 'nature', 4, False, True, (152, 251, 152)),
    Label('sky', 11, 'sky', 5, False, True, (70, 130, 180)),
    Label('person', 12, 'human', 6, True, False, (220, 20, 60)),
    Label('rider', 13, 'human', 6, True, False, (255, 0, 0)),
    Label('car', 14, 'vehicle', 7, True, False, (0, 0, 142)),
    Label('truck', 15, 'vehicle', 7, True, False, (0, 0, 70)),
    Label('bus', 16, 'vehicle', 7, True, False, (0, 60, 100)),
    Label('train', 17, 'vehicle', 7, True, False, (0, 80, 100)),
    Label('motorcycle', 18, 'vehicle', 7, True, False, (0, 0, 230)),
    Label('bicycle', 19, 'vehicle', 7, True, False, (119, 11, 32)),
    Label('static', 20, 'object', 3, True, True, (110, 190, 160)),
    Label('dynamic', 21, 'object', 3, True, False, (170, 120, 50)),
    Label('other', 22, 'object', 3, True, True, (55, 90, 80)),
    Label('water', 23, 'nature', 4, False, True, (45, 60, 150)),
    Label('road lines', 24, 'flat', 1, True, False, (157, 234, 50)),
    Label('ground', 25, 'flat', 1, True, True, (81, 0, 81)),
    Label('bridge', 26, 'construction', 2, True, True, (150, 100, 100)),
    Label('rail track', 27, 'flat', 1, True, True, (230, 150, 140)),
    Label('guard rail', 28, 'construction', 2, True, True, (180, 165, 180)),
    Label('any', 255, 'any', 255, False, False, (0, 0, 0)),
]

# -----------------------------------------------------------------------------
# Create dictionaries for a fast lookup
# -----------------------------------------------------------------------------

# Please refer to the main method below for example usages!

# name to label object
name2label = {label.name: label for label in labels}
# id to label object
id2label = {label.id: label for label in labels}
# category to list of label objects
category2labels = {}
for label in labels:
    category = label.category
    if category in category2labels:
        category2labels[category].append(label)
    else:
        category2labels[category] = [label]
