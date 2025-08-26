# computer-vision/training

This folder contains code to build the YOLO model (RGB) and to detect fire from a lepton camera (IR).

## Configuration

Available environment variables:

- `BASE_MODEL`: The base model to use (default: `yolo11m.pt`)
- `EPOCHS`: The number of training epochs (default: `10`)
- `EXPORT_FORMATS`: The export formats for the trained model (default: None)
  - example: `engine,onnx` for TensorRT and ONNX
- MLFLOW related configuration
  - `ACTIVATE_MLFLOW`: To activate MLFLOW tracking (default: is disabled), you only need to add the environment variable to activate it
  - `MLFLOW_EXPERIMENT_NAME`: The name of the MLFLOW experiment (default: None)
  - `MLFLOW_TRACKING_URI`: The URI of the MLFLOW tracking server (default: None)

## Before you start

To start, download the base dataset to train the model.

First, download the D-fire zip archive and put it in the [`-/computer-vision/training/data/`](./data/) folder, you can find here: [github.com/gaiasd/DFireDataset](https://github.com/gaiasd/DFireDataset).

Then, run the script the following script:

```bash
# assuming you are in the computer-vision folder
bash scripts/extract_dfire_dataset.sh
```

## Training the model

To train the model, you can use the provided Docker Compose file. This will set up a container with all the necessary dependencies.

```bash
# assuming you are in the computer-vision/training/ folder
docker compose up
```

And you should have the dataset `D-Fire` extracted in the `./data/` folder.

## Citations

- For the D-Fire dataset: Pedro Vinícius Almeida Borges de Venâncio, Adriano Chaves Lisboa, Adriano Vilela Barbosa: [An automatic fire detection system based on deep convolutional neural networks for low-power, resource-constrained devices.](https://link.springer.com/article/10.1007/s00521-022-07467-z) In: Neural Computing and Applications, 2022.
