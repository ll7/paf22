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
from data_generation.weights_organizer import WeightsOrganizer
from traffic_light_detection.classification_model import ClassificationModel
from torchvision.transforms import ToTensor


def parse_args():
    """
    Parses arguments for execution given by the command line.
    @return: Parsed arguments
    """
    parser = argparse.ArgumentParser(description='Train traffic light network')
    parser.add_argument('--epochs',
                        default=100,
                        help='number of epochs',
                        type=int)
    parser.add_argument('--num_saves',
                        default=3,
                        help='number of model-weights to be saved',
                        type=int)
    return parser.parse_args()


class TrafficLightTraining:

    def __init__(self, dataset_root, weights_root, device, num_saves):
        """
        Initializes an instance to train a traffic light classification model.
        @param dataset_root: Root directory of the dataset to train on.
            The dataset must have the following structure:
            |-- root
            |   |-- train
            |       |-- class1
            |           |-- image1
            |           |-- image2
            |       |-- class2
            |           |-- image1
            |           |-- image2
            |   |-- val
            |       |-- class1
            |           |-- image1
            |           |-- image2
            |       |-- class2
            |           |-- image1
            |           |-- image2
        @param weights_root: Root directory to store model weights in
        @param device: Device to run training on
        @param num_saves: Number of model-weights to be saved
        """
        self.weights_root = weights_root
        self.device = device
        self.num_saves = num_saves
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
        self.weights_organizer = WeightsOrganizer(model=self.model,
                                                  path=self.weights_root,
                                                  num_saves=self.num_saves)

    def run(self, epochs):
        """
        Trains the model for a given amount of epochs
        @param epochs: number of epochs to train the model on.
            One epoch means to train on every image on the train-subset once.
        """
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
            self.weights_organizer.save(epoch_correct, correct)

    def epoch(self):
        """
        Trains the model for a single epoch
        @return: Average loss and accuracy of the net in this epoch
        """
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
        """
        Executes the model validation on the validation-subset.
        @return: Average loss and accuracy of the net on the validation-subset
        """
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
    root = str(Path(__file__).resolve().parents[2].resolve())
    tr = TrafficLightTraining(root + '/dataset',
                              root + '/model_weights',
                              device,
                              args.num_saves)
    tr.run(args.epochs)
