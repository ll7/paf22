#!/usr/bin/python
#
# Converts the annotations of our generated dataset
# to COCO-style panoptic segmentation format
# (http://cocodataset.org/#format-data).
#
# By default with this tool uses IDs specified in labels.py.
# 'ignoreInEval' categories are removed during the conversion.
#
# In panoptic segmentation format image_id is used to match predictions and
# ground truth.
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

# dataset imports
from labels import id2label, labels


# The main method
def convert2panoptic(datasetPath=None,
                     setNames=["val", "train", "test"]):

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
            datasetPath, "groundtruth", setName, "*", "*.png")
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

        outputBaseFile = "dataset_panoptic_{}".format(setName)
        outFile = os.path.join(datasetPath + "/groundtruth",
                               "{}.json".format(outputBaseFile))
        print("Json file with the annotations in panoptic format will be saved"
              " in {}".format(outFile))

        images = []
        annotations = []
        for progress, f in enumerate(files):
            originalFormat = np.array(Image.open(f))[..., :3]
            panoptic = np.zeros(originalFormat.shape, dtype=np.uint8)

            fileName = os.path.basename(f)
            imageId = fileName.replace("_groundtruth.png", "")
            inputFileName = fileName.replace("_groundtruth.png", ".png")
            # image entry, id for image is its filename without extension
            images.append({"id": imageId,
                           "width": int(originalFormat.shape[1]),
                           "height": int(originalFormat.shape[0]),
                           "file_name": inputFileName})

            formatted = originalFormat.reshape(-1, originalFormat.shape[2])
            segmentIds = np.unique(formatted, axis=0)
            instance_ids = np.zeros((max(segmentIds[:, 0]) + 1),
                                    dtype=np.uint8)
            segmInfo = []
            for segmentId in segmentIds:
                semanticId = segmentId[0]
                labelInfo = id2label[semanticId]
                if labelInfo.hasInstances:
                    instance_id = 1000 * segmentId[0] \
                                  + instance_ids[segmentId[0]]
                    instance_ids[segmentId[0]] += 1
                    isCrowd = 0
                else:
                    instance_id = segmentId[0]
                    isCrowd = 1

                if not labelInfo.hasInstances:
                    isCrowd = 0

                if labelInfo.ignoreInEval:
                    continue
                mask = originalFormat == segmentId
                mask = mask.all(axis=2)
                color = [instance_id % 256, instance_id // 256,
                         instance_id // 256 // 256]
                panoptic[mask] = color
                Image.fromarray(panoptic).save(
                    f.split(".png")[0] + "_instances.png")
                # segment area computation
                area = np.count_nonzero(mask)

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

                segmInfo.append({"id": int(instance_id),
                                 "category_id": int(semanticId),
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
                        dest="datasetPath",
                        help="path to the dataset root folder",
                        default=None,
                        type=str)
    parser.add_argument("--set-names",
                        dest="setNames",
                        help="set names to which apply the function to",
                        nargs='+',
                        default=["val", "train", "test"],
                        type=str)
    args = parser.parse_args()

    convert2panoptic(args.datasetPath, args.setNames)


# call the main
if __name__ == "__main__":
    main()
