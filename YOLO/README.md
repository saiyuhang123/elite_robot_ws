# Fruit Detection Dataset

## Overview

This document describes the custom fruit detection dataset created and annotated for training and evaluating an object detection model (likely YOLO-based). This dataset includes images of the following nine fruit classes: apple, avocado, banana, guava, kiwi, mango, orange, peach, and pineapple.

## Dataset Structure

The dataset is organized into two main directories at the top level:

* **`images/`**: This directory contains all the image files in various formats (e.g., `.jpg`, `.jpeg`, `.png`). The images are of diverse sizes and depict single or multiple instances of the target fruits in various settings and lighting conditions. This directory is further divided into:
    * **`train/`**: Contains images used for training the model (approximately 80% of the total).
    * **`val/`**: Contains images used for validating the model during training (approximately 20% of the total).

* **`labels/`**: This directory contains the annotation files corresponding to the images in the `images/` directory. The annotation files provide the ground truth information about the location and class of each fruit within the images. The annotation format is in **YOLO format** (`.txt` files with the same base name as the image files). This directory is also divided into:
    * **`train/`**: Contains annotation files for the training images.
    * **`val/`**: Contains annotation files for the validation images.

For example, the annotation file for `images/train/apple_1.jpg` would be `labels/train/apple_1.txt`.

## Annotation Format (YOLO)

Each annotation file (`.txt`) in the `labels/` directory corresponds to an image in the `images/` directory with the same base name and contains one line for each detected fruit instance in that image. The format of each line is as follows:

``` <class_id> <x_center> <y_center> <width> <height> ```



Where:

* **`<class_id>`**: An integer representing the class of the fruit. The mapping of class IDs to fruit names in this dataset is:
    * `0`: apple
    * `1`: avocado
    * `2`: banana
    * `3`: guava
    * `4`: kiwi
    * `5`: mango
    * `6`: orange
    * `7`: peach
    * `8`: pineapple
* **`<x_center>`**: The normalized x-coordinate of the center of the bounding box (between 0 and 1).
* **`<y_center>`**: The normalized y-coordinate of the center of the bounding box (between 0 and 1).
* **`<width>`**: The normalized width of the bounding box (between 0 and 1).
* **`<height>`**: The normalized height of the bounding box (between 0 and 1).

**Note:** All coordinates and dimensions are normalized by the image's width and height, respectively. These annotations were created using Label Studio.

## Dataset Statistics

* **Number of Classes:** 9 (apple, avocado, banana, guava, kiwi, mango, orange, peach, pineapple)
* **Total Number of Images:** [Specify the total number of images in the dataset]
    * Training Set: [Approximately 80% of the total]
    * Validation Set: [Approximately 20% of the total]
* **Number of Annotations:** [Specify the total number of fruit annotations across all label files]
* **Image Sizes:** Images are of various original sizes.
* **Distribution of Classes:** [Provide a brief overview of the class distribution in the training set, e.g., "The dataset has a varied distribution of the different fruit classes, with [mention if some classes are more prevalent than others]."]

## Data Preprocessing and Augmentation

**Current Status:** No explicit preprocessing or augmentation has been applied to the images in this dataset.

**Future Steps:** It is recommended to apply appropriate preprocessing steps (e.g., resizing to a consistent input size for the model) and data augmentation techniques (e.g., random crops, flips, rotations, color adjustments) during the training process to improve the model's performance and generalization.

## Intended Use

This dataset is intended for training and evaluating object detection models specifically for identifying the nine listed fruit classes. It can be used for tasks such as:

* Real-time detection of these fruits in images and videos.
* Automated fruit recognition in various applications.
* Benchmarking the performance of object detection models on this specific set of fruits.


## Acknowledgements

I used Label Studio For annotation.

## Contact

Email: aman.kr.ak03@proton.me