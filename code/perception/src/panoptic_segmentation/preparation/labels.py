#!/usr/bin/python

from __future__ import print_function, absolute_import, division
from collections import namedtuple


#--------------------------------------------------------------------------------
# Definitions
#--------------------------------------------------------------------------------

# a label and all meta information
Label = namedtuple( 'Label' , [

    'name'        , # The identifier of this label, e.g. 'car', 'person', ... .
                    # We use them to uniquely name a class

    'id'          , # An integer ID that is associated with this label.
                    # The IDs are used to represent the label in ground truth images
                    # An ID of -1 means that this label does not have an ID and thus
                    # is ignored when creating ground truth images (e.g. license plate).
                    # Do not modify these IDs, since exactly these IDs are expected by the
                    # evaluation server.

    'trainId'     , # Feel free to modify these IDs as suitable for your method. Then create
                    # ground truth images with train IDs, using the tools provided in the
                    # 'preparation' folder. However, make sure to validate or submit results
                    # to our evaluation server using the regular IDs above!
                    # For trainIds, multiple labels might have the same ID. Then, these labels
                    # are mapped to the same class in the ground truth images. For the inverse
                    # mapping, we use the label that is defined first in the list below.
                    # For example, mapping all void-type classes to the same ID in training,
                    # might make sense for some approaches.
                    # Max value is 255!

    'category'    , # The name of the category that this label belongs to

    'categoryId'  , # The ID of this category. Used to create ground truth images
                    # on category level.

    'hasInstances', # Whether this label distinguishes between single instances or not

    'ignoreInEval', # Whether pixels having this class as ground truth label are ignored
                    # during evaluations or not

    'color'       , # The color of this label
    ] )


#--------------------------------------------------------------------------------
# A list of all labels
#--------------------------------------------------------------------------------

# Please adapt the train IDs as appropriate for your approach.
# Note that you might want to ignore labels with ID 255 during training.
# Further note that the current train IDs are only a suggestion. You can use whatever you like.
# Make sure to provide your results using the original IDs and not the training IDs.
# Note that many IDs are ignored in evaluation and thus you never need to predict these!

labels = [
    #       name                     id    trainId   category            catId     hasInstances   ignoreInEval   color
    Label(  'unlabeled'            ,  0 ,        0 , 'void'            , 0       , False        , True         , (  0,  0,  0) ),
    Label(  'building'             ,  1 ,        1 , 'construction'    , 1       , False        , False        , ( 70, 70, 70) ),
    Label(  'fence'                ,  2 ,        2 , 'construction'    , 1       , False        , False        , (100, 40, 40) ),
    Label(  'other'                ,  3 ,        3 , 'other'           , 2       , False        , False        , ( 55, 90, 80) ),
    Label(  'pedestrian'           ,  4 ,        4 , 'human'           , 3       , True         , False        , (220, 20, 60) ),
    Label(  'pole'                 ,  5 ,        5 , 'construction'    , 1       , False        , False        , (153,153,153) ),
    Label(  'road line'            ,  6 ,        6 , 'flat'            , 5       , False        , False        , (157,234, 50) ),
    Label(  'road'                 ,  7 ,        7 , 'flat'            , 5       , False        , False        , (128, 64,128) ),
    Label(  'sidewalk'             ,  8 ,        8 , 'flat'            , 5       , False        , False        , (244, 35,232) ),
    Label(  'vegetation'           ,  9 ,        9 , 'nature'          , 6       , False        , False        , (107,142, 35) ),
    Label(  'vehicle'              , 10 ,       10 , 'vehicle'         , 7       , True         , False        , (  0,  0,142) ),
    Label(  'wall'                 , 11 ,       11 , 'construction'    , 1       , False        , False        , (102,102,156) ),
    #Label(  'traffic sign'         , 12 ,       12 , 'object'          , 8       , False        , False        , (220,220,  0) ),
    Label(  'sky'                  , 13 ,       13 , 'sky'             , 9       , False        , False        , ( 70,130,180) ),
    Label(  'ground'               , 14 ,       14 , 'flat'            , 5       , False        , True         , ( 81,  0, 81) ),
    Label(  'bridge'               , 15 ,       15 , 'construction'    , 1       , False        , False        , (150,100,100) ),
    Label(  'rail track'           , 16 ,       16 , 'flat'            , 5       , False        , False        , (230,150,140) ),
    Label(  'guard rail'           , 17 ,       17 , 'construction'    , 1       , False        , False        , (180,165,180) ),
    #Label(  'traffic light'        , 18 ,       18 , 'object'          , 8       , False        , False        , (250,170, 30) ),
    Label(  'static'               , 19 ,       19 , 'static'          , 10      , False        , True         , (110,190,160) ),
    Label(  'dynamic'              , 20 ,       20 , 'dynamic'         , 11      , False        , True         , (170,120, 50) ),
    Label(  'water'                , 21 ,       21 , 'nature'          , 6       , False        , False        , ( 45, 60,150) ),
    Label(  'terrain'              , 22 ,       22 , 'nature'          , 6       , False        , False        , (145,170,100) )
]


#--------------------------------------------------------------------------------
# Create dictionaries for a fast lookup
#--------------------------------------------------------------------------------

# Please refer to the main method below for example usages!

# name to label object
name2label      = { label.name    : label for label in labels           }
# id to label object
id2label        = { label.id      : label for label in labels           }
# trainId to label object
trainId2label   = { label.trainId : label for label in reversed(labels) }
# category to list of label objects
category2labels = {}
for label in labels:
    category = label.category
    if category in category2labels:
        category2labels[category].append(label)
    else:
        category2labels[category] = [label]

#--------------------------------------------------------------------------------
# Assure single instance name
#--------------------------------------------------------------------------------

# returns the label name that describes a single instance (if possible)
# e.g.     input     |   output
#        ----------------------
#          car       |   car
#          cargroup  |   car
#          foo       |   None
#          foogroup  |   None
#          skygroup  |   None
def assureSingleInstanceName( name ):
    # if the name is known, it is not a group
    if name in name2label:
        return name
    # test if the name actually denotes a group
    if not name.endswith("group"):
        return None
    # remove group
    name = name[:-len("group")]
    # test if the new name exists
    if not name in name2label:
        return None
    # test if the new name denotes a label that actually has instances
    if not name2label[name].hasInstances:
        return None
    # all good then
    return name
