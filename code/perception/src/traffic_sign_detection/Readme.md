# Traffic Sign Detection

The training code is inspired by [this tutorial](https://debuggercafe.com/traffic-sign-recognition-using-pytorch-and-deep-learning/).

## Run trainig

```shell
b5 shell
cd perception/src/traffic_sign_detection/
python src/train.py
```

## Dataset

Since there doesn't exist a large dataset for CARLA traffic signs
the [German Traffic Sign Recognition Benchmark](https://benchmark.ini.rub.de/gtsrb_news.html)
was used.

Even if these are real images, they look pretty similar to the CARLA ones.