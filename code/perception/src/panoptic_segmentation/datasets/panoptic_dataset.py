import os
import json
import torch
import numpy as np
from PIL import Image
from torch.utils.data import Dataset
import torchvision.transforms.functional as F
from detectron2.structures import Instances, BitMasks, Boxes


class PanopticDataset(Dataset):
    """A dataset for Panoptic task"""

    def __init__(self, path_json, root_dir, split, transform=None):
        """
        Args:
            path_json (string): Path to the json file with annotations.
            root_dir (string): Directory with all the images.
            transform (callable, optional): Optional transform to be applied
                on a sample.
        """
        self.root_dir = root_dir
        self.split = split
        self.transform = transform
        # Load json file containing information about the dataset
        path_file = os.path.join(self.root_dir, path_json)
        data = json.load(open(path_file))
        annotations = data['annotations']
        images = data['images']
        self.categories = data['categories']
        # TODO Possible problem with VOID label and train_id
        # Add mapper to training id and class id
        self.categories = sorted(self.categories, key=lambda d: d["isthing"])
        index = self.categories.index(
            next(x for x in self.categories if x["isthing"] == 1))
        self.semantic_class_mapper = {cat['id']: {
            'train_id': i,
            'isthing': cat['isthing']
        }
            for i, cat in enumerate(self.categories)}
        self.instance_class_mapper = {cat['id']: {
            'train_id': i
        }
            for i, cat in enumerate(self.categories[index:])}
        self.semantic_class_mapper.update({0: {
            'train_id': len(self.categories) + 1,
            'isthing': 0}})
        self.semantic_train_id_to_eval_id = [7, 8, 11, 12, 13, 17, 19, 20, 21,
                                             22, 23, 24, 25, 26, 27, 28, 31,
                                             32, 33, 0]
        self.instance_train_id_to_eval_id = [24, 25, 26, 27, 28, 31, 32, 33]

        # Generate a dictionary with all needed information with idx as key
        self.meta_data = {}
        for i in range(len(images)):
            self.meta_data.update({i: {}})
            # TODO Error Message
            assert annotations[i]['image_id'] == images[i]['id']
            self.meta_data[i].update({
                'labelfile_name': annotations[i]['file_name']
            })
            self.meta_data[i].update(annotations[i])
            self.meta_data[i].update(images[i])

    def __len__(self):
        return len(self.meta_data)

    def __getitem__(self, idx):
        # Retrieve meta data of image
        img_data = self.meta_data[idx]

        # Load image
        path_img = os.path.join(self.root_dir,
                                img_data["image_id"].split("_")[1],
                                self.split,
                                img_data['file_name'].split('_')[0],
                                img_data['file_name'].replace('gtFine_', ''))
        image = np.asarray(Image.open(path_img))[:, :, :3]

        # Get label info
        path_label = os.path.join(self.root_dir,
                                  'groundtruth',
                                  self.split,
                                  img_data['file_name'].split('_')[0],
                                  img_data['labelfile_name'])
        panoptic = np.asarray(Image.open(path_label))[:, :, :3]
        panoptic = rgb2id(panoptic, self.categories)

        # Get bbox info
        rpn_bbox = []
        class_bbox = []
        for seg in img_data['segments_info']:
            seg_category = self.semantic_class_mapper[seg['category_id']]
            if seg_category['isthing']:
                rpn_bbox.append(seg["bbox"])
                class_bbox.append(
                    self.instance_class_mapper[seg['category_id']])

        # Apply augmentation with albumentations
        if self.transform is not None:
            transformed = self.transform(
                image=image,
                mask=panoptic,
                bboxes=rpn_bbox,
                class_labels=class_bbox
            )
            image = transformed['image']
            panoptic = transformed['mask']
            rpn_bbox = transformed['bboxes']
            # class_bbox = transformed['class_labels']

        # Create instance class for detectron (Mask RCNN Head)
        instance = Instances(panoptic.shape)

        # Create semantic segmentation target with augmented data
        semantic = np.zeros_like(panoptic, dtype=np.longlong)
        rpn_mask = np.zeros_like(panoptic)
        instance_mask = []
        instance_cls = []

        for seg in img_data['segments_info']:
            seg_category = self.semantic_class_mapper[seg['category_id']]
            semantic[panoptic == seg["id"]] = seg_category['train_id']
            # If segmentation is a thing generate a mask for maskrcnn target
            # Collect information for RPN targets
            if seg_category['isthing']:
                seg_category = self.instance_class_mapper[seg['category_id']]
                mask = np.zeros_like(panoptic)
                mask[panoptic == seg["id"]] = 1  # seg_category['train_id']
                instance_cls.append(seg_category['train_id'])
                instance_mask.append(mask)
                # RPN targets
                rpn_mask[panoptic == seg["id"]] = 1

        # Create same size of bbox and mask instance
        if len(rpn_bbox) > 0:
            rpn_bbox = coco_to_pascal_bbox(np.stack([*rpn_bbox]))

            instance.gt_masks = BitMasks(instance_mask)
            instance.gt_classes = torch.as_tensor(instance_cls)
            instance.gt_boxes = Boxes(rpn_bbox)
        else:
            instance.gt_masks = BitMasks(torch.Tensor([]).view(0, 1, 1))
            instance.gt_classes = torch.as_tensor([])
            instance.gt_boxes = Boxes([])

        return {
            'image': np.array(image),
            'semantic': semantic,
            'instance': instance,
            'image_id': img_data['image_id']
        }


def rgb2id(color, categories):
    """ Pass the image from RGB to the instance id value
    See COCO format doc https://cocodataset.org/#format-data
    """
    panoptic = np.zeros((color.shape[:2]), dtype=np.uint32)
    formatted = color.reshape(-1, color.shape[2])
    segmentIds = np.unique(formatted, axis=0)
    instance_ids = np.zeros((max(segmentIds[:, 0]) + 1), dtype=np.uint8)
    for segmentId in segmentIds:
        semanticId = segmentId[0]
        labelInfo = next(item for item in categories if item["id"] == semanticId)
        indices = np.where(np.all(color == segmentId, axis=-1))
        coords = zip(indices[0], indices[1])
        if labelInfo["isthing"]:
            instance_id = 1000 * segmentId[0] + instance_ids[
                segmentId[0]]
            instance_ids[segmentId[0]] += 1
        else:
            instance_id = segmentId

        coords = list(map(list, list(coords)))
        for coord in coords:
            if coord[0] > 719:
                pass
            panoptic[coord[0], coord[1]] = instance_id
    return panoptic


def coco_to_pascal_bbox(bbox):
    return np.stack((bbox[:, 0], bbox[:, 1],
                     bbox[:, 0] + bbox[:, 2], bbox[:, 1] + bbox[:, 3]), axis=1)


def collate_fn(inputs):
    return {
        'image': torch.stack([F.to_tensor(i['image']) for i in inputs]),
        'semantic': torch.as_tensor([i['semantic'] for i in inputs]),
        'instance': [i['instance'] for i in inputs],
        'image_id': [i['image_id'] for i in inputs]
    }
