# computer-vision

This folder contains code for the computer vision module (focused on detecting fire or hot objects) and the decision-making algorithm used in a fire hazard detection system and autonomous robot.

## Before you start

To start, download the base dataset to train the model.

First, download the D-fire zip archive and put it in the [`-/computer-vision/data/`](./computer-vision/data/) folder, you can find here: [github.com/gaiasd/DFireDataset](https://github.com/gaiasd/DFireDataset).

Then, run the script to format it correctly:

```bash
# assuming you are in the computer-vision folder
bash scripts/format_dfire_dataset.sh
```

## Citations

- For the D-Fire dataset: Pedro Vinícius Almeida Borges de Venâncio, Adriano Chaves Lisboa, Adriano Vilela Barbosa: [An automatic fire detection system based on deep convolutional neural networks for low-power, resource-constrained devices.](https://link.springer.com/article/10.1007/s00521-022-07467-z) In: Neural Computing and Applications, 2022.
