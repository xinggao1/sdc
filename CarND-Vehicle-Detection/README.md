# Vehicle Detection Project

## Goals / steps

* Perform a Histogram of Oriented Gradients (HOG) feature extraction on a labeled training set of images and train a classifier Linear SVM classifier
* Apply a color transform and append binned color features, as well as histograms of color, to your HOG feature vector.
* For those first two steps, normalize features and randomize a selection for training and testing.
* Implement a sliding-window technique and use trained classifier to search for vehicles in images.
* Run pipeline on a video stream
* Estimate a bounding box for vehicles detected.

[//]: # (Image References)
[image1]: ./output_images/car_not_car.png
[image2]: ./output_images/HOG_example.png
[image3]: ./output_images/sliding_windows.png
[image4]: ./output_images/sliding_window.png
[image5]: ./output_images/heat.png
[image6]: ./output_images/heat2.png
[image8]: ./output_images/normalize.png

[video1]: ./project_video.mp4

### Histogram of Oriented Gradients (HOG)

#### 1. Extract HOG features from the training images.

The code for this step is contained in the 5th code cell of the IPython notebook 
called `vehicleDetection.ipynb`  

I started by reading in all the `vehicle` and `non-vehicle` images.  
Here is an example of one of each of the `vehicle` and `non-vehicle` classes:

![alt text][image1]

I then explored different color spaces and different `extract_features()` parameters 
(`cspace`, `hog_channel`,`orient`, `pixels_per_cell`, and `cells_per_block`).  I grabbed random images 
from each of the two classes and displayed them to get a feel for what the `extract_features()` output looks like.

Here is an example using the `YCrCb` color space and HOG parameters of `orientations=8`,
 `pixels_per_cell=(8, 8)` and `cells_per_block=(2, 2)`:
 
![alt text][image2]

YCrCb 3-channel HOG features plus spatially binned color and histograms of color are used. And all the features 
 are normalized.

![alt text][image8]

#### 2. Settle on final choice of HOG parameters.

I tried various combinations of parameters and the following performs best in the classifier training
```python
pix_per_cell = 8
cell_per_block = 2
orient = 9
colorspace = 'YCrCb' 
hog_channel = "ALL" 
```

#### 3. Train a classifier

I trained a linear SVM using linear supporting vector machine `LinearSVC()`.
The feature vector from HOG has length 5292. The final test Accuracy of SVC is 98%.

The code for this step is contained in the 16th code cell of the IPython notebook 
called `vehicleDetection.ipynb`  

### Sliding Window Search

#### 1. Implemented a sliding window search

I used two sizes of window 96x96 and 128x128 with overlap 0.7 and 0.8. 
And 96x96 windows are positioned further where cars appear smaller, while 128x128 windows are positioned closer.
More windows size could be used, if more computing power is available.

![alt text][image3]

#### 2. Examples of test images

Ultimately I searched on two scales using YCrCb 3-channel HOG features plus spatially
 binned color and histograms of color in the feature vector, which provided a nice result.  
 Here are some example images:

![alt text][image4]
---

### Video Implementation

#### 1. Final video output.
Here's a [link to my video result](./project_video_out.mp4)


#### 2. Implemented filter for false positives and method for combining overlapping bounding boxes.

I recorded the positions of positive detections in each frame of the video.  From the positive detections I created a 
heatmap and then thresholded that map to identify vehicle positions.  I then used `scipy.ndimage.measurements.label()` 
to identify individual blobs in the heatmap.  I then assumed each blob corresponded to a vehicle.  
I constructed bounding boxes to cover the area of each blob detected.  

Here's an example result showing the heatmap from a series of frames of video, 
the result of `scipy.ndimage.measurements.label()` and the bounding boxes then overlaid on the last frame of video:

### Here are 5 frames, their corresponding heatmaps, and integrated heatmap:

![alt text][image5]


### Here the resulting bounding boxes are drawn onto the last frame in the series:
![alt text][image6]



---

###Discussion

#### Problems / issues

The detection accuracy was not good enough with YCrCb 3-channel HOG features only, so I added binned color and histograms of color
to improve.
The pipeline might fail when there are distracting objects on the side, integrating the lane detection will help.
