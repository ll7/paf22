#!/usr/bin/python
#
# Converts the *instanceIds.png annotations of the Cityscapes dataset
# to COCO-style panoptic segmentation format
# (http://cocodataset.org/#format-data).
# The convertion is working for 'fine' set of the annotations.
#
# By default with this tool uses IDs specified in labels.py. You can use flag
# --use-train-id to get train ids for categories. 'ignoreInEval' categories are
# removed during the conversion.
#
# In panoptic segmentation format image_id is used to match predictions and
# ground truth.
# For cityscapes image_id has form <city>_123456_123456 and corresponds to the
# prefix of cityscapes image files.
#

# python imports
from __future__ import print_function, absolute_import, division, \
    unicode_literals
import os
import glob
import sys
import argparse
import json
import numpy as np

# Image processing
from PIL import Image

# cityscapes imports
from labels import id2label, labels


# The main method
def convert2panoptic(cityscapesPath=None,
                     outputFolder=None,
                     setNames=["val", "train", "test"]):
    # Where to look for Cityscapes
    if cityscapesPath is None:
        if 'CITYSCAPES_DATASET' in os.environ:
            cityscapesPath = os.environ['CITYSCAPES_DATASET']
        else:
            cityscapesPath = os.path.join(
                os.path.dirname(os.path.realpath(__file__)), '..', '..')
        cityscapesPath = os.path.join(cityscapesPath, "gtFine")

    if outputFolder is None:
        outputFolder = cityscapesPath

    categories = []
    for label in labels:
        if label.ignoreInEval:
            continue
        categories.append({'id': int(label.id),
                           'name': label.name,
                           'color': label.color,
                           'supercategory': label.category,
                           'isthing': 1 if label.hasInstances else 0})

    for setName in setNames:
        # how to search for all ground truth
        searchFine = os.path.join(
            cityscapesPath, "groundtruth", setName, "*", "*.png")
        # search files
        filesFine = glob.glob(searchFine)
        filesFine.sort()

        files = filesFine
        # quit if we did not find anything
        if not files:
            print(
                "Did not find any files for {} set using matching pattern {}"
                .format(setName, searchFine)
            )
            exit(1)
        # a bit verbose
        print("Converting {} annotation files for {} set."
              .format(len(files), setName))

        outputBaseFile = "cityscapes_panoptic_{}".format(setName)
        outFile = os.path.join(outputFolder, "{}.json".format(outputBaseFile))
        print("Json file with the annotations in panoptic format will be saved"
              " in {}".format(outFile))

        images = []
        annotations = []
        for progress, f in enumerate(files):

            originalFormat = np.array(Image.open(f))[..., :3]

            fileName = os.path.basename(f)
            imageId = fileName.replace("_groundtruth.png", "")
            inputFileName = fileName.replace("_groundtruth.png", ".png")
            # image entry, id for image is its filename without extension
            images.append({"id": imageId,
                           "width": int(originalFormat.shape[1]),
                           "height": int(originalFormat.shape[0]),
                           "file_name": inputFileName})

            segmentIds = np.unique(originalFormat.reshape(
                -1, originalFormat.shape[2]), axis=0)
            segmInfo = []
            for segmentId in segmentIds:
                semanticId = segmentId[0]
                dic = categories[semanticId - 1] if semanticId < 255 \
                    else len(categories) - 1
                isCrowd = 1 if dic["isthing"] == 0 else 0
                labelInfo = id2label[semanticId]
                categoryId = labelInfo.id

                mask = originalFormat == segmentId
                mask = np.all(mask, axis=2)
                # segment area computation
                area = np.sum(mask)

                # bbox computation for a segment
                hor = np.sum(mask, axis=0)
                hor_idx = np.nonzero(hor)[0]
                x = hor_idx[0]
                width = hor_idx[-1] - x + 1
                vert = np.sum(mask, axis=1)
                vert_idx = np.nonzero(vert)[0]
                y = vert_idx[0]
                height = vert_idx[-1] - y + 1
                bbox = [int(x), int(y), int(width), int(height)]

                segmInfo.append({"id": int(semanticId),
                                 "category_id": int(categoryId),
                                 "area": int(area),
                                 "bbox": bbox,
                                 "iscrowd": isCrowd})

            annotations.append({'image_id': imageId,
                                'file_name': fileName,
                                "segments_info": segmInfo})

            print("\rProgress: {:>3.2f} %"
                  .format((progress + 1) * 100 / len(files)), end=' ')
            sys.stdout.flush()

        print("\nSaving the json file {}".format(outFile))
        d = {'images': images,
             'annotations': annotations,
             'categories': categories}
        with open(outFile, 'w') as f:
            json.dump(d, f, sort_keys=True, indent=4)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-folder",
                        dest="cityscapesPath",
                        help="path to the Cityscapes dataset 'gtFine' folder",
                        default=None,
                        type=str)
    parser.add_argument("--output-folder",
                        dest="outputFolder",
                        help="path to the output folder.",
                        default=None,
                        type=str)
    parser.add_argument("--set-names",
                        dest="setNames",
                        help="set names to which apply the function to",
                        nargs='+',
                        default=["val", "train", "test"],
                        type=str)
    args = parser.parse_args()

    convert2panoptic(args.cityscapesPath, args.outputFolder, args.setNames)


# call the main
if __name__ == "__main__":
    # dir = "center"
    # folder = "/workspace/dataset/test_data/rgb/" + dir + "/"
    #
    # for filename in os.listdir(folder):
    #     new = "2_" + dir + "_" + filename.split(".")[0] + ".png"
    #     source = folder + filename
    #     dest = "/workspace/dataset/test_data/" + dir + "/" + new
    #     os.rename(source, dest)

    main()
