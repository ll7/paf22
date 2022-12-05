from datetime import datetime
import os
import torch


class WeightsOrganizer:

    def __init__(self, model, path, num_saves):
        """
        Initializes a weights organizer for the given model.
        @param model: Classification model that has to be saved
        @param path: Path to save the model weights in
        @param num_saves: Number of model-weights to be saved
        """
        self.model = model
        self.num_saves = num_saves
        self.best = []
        self.time = datetime.now().strftime("%d.%m.%Y_%H.%M")
        self.path = os.path.join(path + "/", self.time + "/")
        os.mkdir(self.path)

    def save(self, accuracy, val_accuracy):
        """
        Saves the model when validation-accuracy and/or accuracy has improved
        @param accuracy: Accuracy of the model in the last epoch
        @param val_accuracy: Accuracy of the model on the validation-subset
        """
        filename = self.path + f"model_acc_{round(accuracy, 2)}" \
                             + f"_val_{round(val_accuracy, 2)}.pt"
        if len(self.best) == 0:
            torch.save(self.model.state_dict(), filename)
            self.best.append((accuracy, val_accuracy, filename))
        elif val_accuracy > self.best[len(self.best) - 1][1] or \
            (val_accuracy >= self.best[len(self.best) - 1][1] and
             accuracy > self.best[len(self.best) - 1][0]):

            if len(self.best) == self.num_saves:
                delete = self.best[0][2]
                self.best.pop(0)
                os.remove(delete)
            torch.save(self.model.state_dict(), filename)
            self.best.append((accuracy, val_accuracy, filename))
            self.best = sorted(self.best, key=lambda t: (t[1], t[0]))