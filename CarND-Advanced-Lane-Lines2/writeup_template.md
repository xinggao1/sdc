## Advanced Lane Finding Project

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[image1]: ./output_images/calibresult.png "Undistorted calibration"
[image2]: ./output_images/test_images1_undistort.png "undistorted test image"
[image3]: ./output_images/binary_threshHold.png "threshhold binary"
[image4]: ./output_images/test_image_perspective.png "perspective"
[image5]: ./output_images/warped.png "warped"
[image6]: ./output_images/warped1.png "warped1"
[image7]: ./output_images/lane_follow.png "lane_follow"
[image8]: ./output_images/lane_start.png "lane_start"
[image9]: ./output_images/lane_on_road.png "lane_on_road"
[image10]:./test_images/straight_lines2.jpg "straight_lines2"
[video1]: ./project_video_out.mp4

### Camera Calibration

#### 1. Briefly state how you computed the camera matrix and distortion coefficients. Provide an example of a distortion corrected calibration image.

The code for this step is contained in the first code cell of the IPython notebook located in 
`advancedLaneFind.ipynb`.  

I start by preparing "object points", which will be the (x, y, z) coordinates of the chessboard corners in the world.
 Here I am assuming the chessboard is fixed on the (x, y) plane at z=0, such that the object points are the same for
 each calibration image.  Thus, `objp` is just a replicated array of coordinates, and `objpoints` will be appended 
 with a copy of it every time I successfully detect all chessboard corners in a test image.  `imgpoints` will be 
 appended with the (x, y) pixel position of each of the corners in the image plane with each successful chessboard 
 detection.  

I then used the output `objpoints` and `imgpoints` to compute the camera calibration and distortion coefficients
 using the `cv2.calibrateCamera()` function.  I applied this distortion correction to the test image using the 
 `cv2.undistort()` function and obtained this result: 

![alt text][image1]

### Pipeline (single images)

#### 1. Provide an example of a distortion-corrected image.


To demonstrate this step, I will describe how I crop only bottom half, and apply the distortion correction to one of the 
test images like this one:

![alt text][image10]

![alt text][image2]

#### 2. Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image.  Provide an example of a binary image result.

I used a combination of color and gradient thresholds to generate a binary image (thresholding steps at 4th cell of
`advancedLaneFind.ipynb`.). The S channel of HLS and R channel of RGB image are taken to apply Sobel filter. And then the
filter results are combined.
Here's an example of my output for this step. 

![alt text][image3]

#### 3. Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.

The code for my perspective transform includes a function called `persTranIMG()`, which appears in  5th cell of
`advancedLaneFind.ipynb`  The `persTranIMG()` function takes as inputs an image (`img`), use source (`srcP`) and destination (`dstP`) points. 
 I chose the hardcode the source and destination points in the following manner:

```python
srcP = np.float32(
    [[(img_size[0] / 2) - 25,  75],
    [((img_size[0] / 6) + 42), img_size[1]],
    [(img_size[0] * 5 / 6 + 15) , img_size[1]],
    [(img_size[0] / 2 + 41),  75]])
dstP = np.float32(
    [[(img_size[0] / 4), 0],
    [(img_size[0] / 4), img_size[1]*ratio],
    [(img_size[0] * 3 / 4), img_size[1]*ratio],
    [(img_size[0] * 3 / 4), 0]])
```
This resulted in the following source and destination points:

| Source        | Destination   | 
|:-------------:|:-------------:| 
|  581,    75     | 303, 0        | 
| 244,   314      | 303, 628      |
| 1025, 314     | 909, 628      |
| 647 ,75    | 909, 0        |

I verified that my perspective transform was working as expected by drawing the `srcP` and `dst` points onto a test 
image and its warped counterpart to verify that the lines appear parallel in the warped image.

![alt text][image4]

![alt text][image5]

#### 4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?

Then I did some other stuff and fit my lane lines with a 2nd order polynomial kinda like this:

By sweeping the window, all the potential lane points are extracted for both left and right. 
The code for this process is in the function `fitPolyline_start()` in 7th cell of `advancedLaneFind.ipynb`
![alt text][image8]

After the first poly line, the following search is based on the area near last lane line.
The code for this process is in the function `fitPolyline_follow()` in 8th cell of `advancedLaneFind.ipynb`
![alt text][image7]

#### 5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

I did this in function `calCurv()` in 9th cell of `advancedLaneFind.ipynb`

```python
left_curverad = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
```

#### 6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.

I implemented this in function `drawLane()` in 10th cell of `advancedLaneFind.ipynb`.
Here is an example of my result on a test image, the lane calculated based on the warped lane mark is warped back to the 
original image:

![alt text][image6]

![alt text][image9]

---

### Pipeline (video)


Here's a [link to my video result](./project_video_out.mp4)

---

### Discussion

Frist of all, my pipe line is based a clear view in front the vehicle. If there is cars in front, which is likely in busy 
hours. My pipe might fail. To improve it, approaches including varying the distance looked ahead should help.
 
Another issue with my pipe line is that, the car is approximately in the center of the lane. So when the car change lanes, 
 a different pipe will be required.
