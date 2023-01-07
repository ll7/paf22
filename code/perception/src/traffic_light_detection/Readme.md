# Traffic light detection

The training is setup as DVC experiment. More details about this can be found [here](../../../doc/02_development/11_dvc.md)

## Training

The training can be run with `dvc exp run`.
The params defined in `params.yaml` can be overridden via commandline with e.g. `dvc exp run --set-param train.epochs=24`.
