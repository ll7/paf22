import os
from datetime import datetime
from pathlib import Path
import torch


class TrafficLightConfig:

    def __init__(self):
        self.NUM_SAVES = 3  # Amount of models to save in the current training
        self.DEVICE = ('cuda' if torch.cuda.is_available() else 'cpu')
        self.TIME = datetime.now().strftime("%d.%m.%Y_%H.%M")

        self.ROOT_PATH = str(Path(__file__).resolve().parents[1].resolve())
        self.DATASET_PATH = self.ROOT_PATH + "/dataset"
        self.WEIGHTS_PATH = self.ROOT_PATH + "/model_weights/"
        self.WEIGHTS_PATH = os.path.join(self.WEIGHTS_PATH, self.TIME + "/")

        # Amount of epochs to train
        # One epoch: Training with all images from training dataset once
        self.EPOCHS = 100
        self.BATCH_SIZE = 32
        self.NUM_WORKERS = 4
        self.NUM_CLASSES = 4  # Traffic light states: green, yellow, red, back
        self.NUM_CHANNELS = 3  # RGB encoded images
