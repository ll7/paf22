import argparse
import torch.cuda
import torchvision.transforms as t
from tqdm import tqdm
from torch.optim.adam import Adam
from torch.utils.data import DataLoader
from torchvision.datasets import ImageFolder
from pathlib import Path
from data_generation.transforms import Normalize, ResizeAndPadToSquare, \
    load_image
from traffic_light_detection.ClassificationModel import ClassificationModel
from torchvision.transforms import ToTensor
import os


def parse_args():
    parser = argparse.ArgumentParser(description='Train traffic light network')
    parser.add_argument('--epochs',
                        default=100,
                        help='number of epochs',
                        type=int)

    parser.add_argument('--overwrite',
                        default=False,
                        help='should this training overwrite existing weights',
                        type=bool)
    return parser.parse_args()


class TrafficLightTraining:

    def __init__(self, dataset_root, device, overwrite):
        self.overwrite = overwrite
        self.device = device
        train_transforms = t.Compose([
            ToTensor(),
            ResizeAndPadToSquare([32, 32]),
            Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])
            # ApplyMask(dataset_root + "/mask.png")
        ])
        val_transforms = t.Compose([
            ToTensor(),
            ResizeAndPadToSquare([32, 32]),
            Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])
        ])
        self.train_dataset = ImageFolder(root=dataset_root + "/train",
                                         transform=train_transforms,
                                         loader=load_image)
        self.train_loader = DataLoader(dataset=self.train_dataset,
                                       batch_size=32,
                                       num_workers=4,
                                       shuffle=True)
        self.val_dataset = ImageFolder(root=dataset_root + "/val",
                                       transform=val_transforms,
                                       loader=load_image)
        self.val_loader = DataLoader(dataset=self.val_dataset,
                                     batch_size=32,
                                     num_workers=4)
        self.model = ClassificationModel(num_classes=4, in_channels=3)
        self.optimizer = Adam(self.model.parameters())
        self.loss_function = torch.nn.CrossEntropyLoss()

    def run(self, epochs):
        maximum_val = 0
        maximum_train = 0
        self.model.to(self.device)
        tepoch = tqdm(range(epochs))
        for i in range(epochs):
            if i > 50:
                for g in self.optimizer.param_groups:
                    g['lr'] = 0.0001
            elif i > 90:
                for g in self.optimizer.param_groups:
                    g['lr'] = 0.00001
            epoch_loss, epoch_correct = self.epoch()
            loss, correct = self.validate()
            tepoch.set_postfix(loss=epoch_loss, accuracy=epoch_correct,
                               val_loss=loss, val_accuracy=correct)
            tepoch.update(1)

            if self.overwrite and \
                (correct > maximum_val or
                 (correct >= maximum_val and epoch_correct > maximum_train)):
                path = str(Path(__file__).parents[2].resolve()) \
                       + "/model_weights/"
                filelist = [f for f in os.listdir(path)]
                for f in filelist:
                    os.remove(os.path.join(path, f))
                torch.save(self.model.state_dict(), path
                           + f"model_acc_{round(epoch_correct, 2)}"
                           + f"_val_{round(correct, 2)}")
                maximum_val = correct
                maximum_train = epoch_correct \
                    if epoch_correct > maximum_train else maximum_train

    def epoch(self):
        self.model.train()
        epoch_loss = 0
        epoch_correct = 0
        for i, data in enumerate(self.train_loader):
            images = data[0].to(self.device)
            labels = data[1].to(self.device)
            self.optimizer.zero_grad()

            outputs = self.model(images)
            loss = self.loss_function(outputs, labels)
            epoch_loss += loss.item()
            _, predictions = torch.max(outputs.data, 1)
            corr = (predictions == labels)
            epoch_correct += corr.sum().item()

            loss.backward()
            self.optimizer.step()
        epoch_loss /= len(self.train_dataset)
        epoch_correct /= len(self.train_dataset)
        return epoch_loss, 100. * epoch_correct

    def validate(self):
        self.model.eval()
        val_loss = 0.
        val_correct = 0
        for i, data in enumerate(self.val_loader):
            images = data[0].to(self.device)
            labels = data[1].to(self.device)

            with torch.no_grad():
                outputs = self.model(images)
            loss = self.loss_function(outputs, labels)
            val_loss += loss.item()
            _, predictions = torch.max(outputs.data, 1)
            corr = (predictions == labels)
            val_correct += corr.sum().item()

        val_loss /= len(self.val_dataset)
        val_correct /= len(self.val_dataset)
        return val_loss, 100. * val_correct


if __name__ == '__main__':
    args = parse_args()
    device = ('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Computation device: {device}\n")
    root = Path(__file__).parents[2]
    tr = TrafficLightTraining(str(root.resolve())
                              + '/dataset',
                              device, args.overwrite)
    tr.run(args.epochs)
