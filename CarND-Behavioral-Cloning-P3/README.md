# Behavioral Cloning


###  The goals and steps
* Use the simulator to collect data of good driving behavior
* Build, a convolution neural network in Keras that predicts steering angles from images
* Train and validate the model with a training and validation set
* Test that the model successfully drives around track one without leaving the road


[//]: # (Image References)

[image1]: ./img/1.png "Model Visualization"
[image2]: ./img/2.jpg "center drive"
[image3]: ./img/3.jpg "Recovery Image"
[image4]: ./img/4.jpg "Recovery Image"
[image5]: ./img/5.jpg "Recovery Image"
[image6]: ./img/6.png "Normal Image"
[image7]: ./img/7.png "Flipped Image"


---

#### 1.Project files:
* model.py containing the script to create and train the model
* drive.py for driving the car in autonomous mode
* model.h5 containing a trained convolution neural network 
* writeup_report.md or writeup_report.pdf summarizing the results

#### 2. Functional code
Using the Udacity provided simulator and my drive.py file, the car can be driven autonomously around the track by executing 
```sh
python drive.py model.h5
```

#### 3. Pipeline

The model.py file contains the code for training and saving the convolution neural network.
The file shows the pipeline I used for training and validating the model, and it contains
comments to explain how the code works.

### Model Architecture and Training Strategy

#### 1. Model architecture

My model follows the nVidia network, consists of 5 convolution layers with 
 5x5 and 3x3 filter sizes and depths between 24 and 64 (model.py lines 130-140) 

The model includes RELU activation to introduce nonlinearity (code line 130), 
and the data is normalized in the model using a Keras lambda layer (code line 130). 

#### 2. Attempts to reduce overfitting

The model use a train data set of 30864, as large as resource permits to prevent overfitting.

The model was trained and validated on different data sets to ensure that the model was not overfitting
 (code line 144). The model was tested by running it through the simulator and ensuring that the vehicle could stay on the track.

#### 3. Model parameter tuning

The model used an adam optimizer, so the learning rate was not tuned manually (model.py line 142).

#### 4. Appropriate training data

Training data was chosen to keep the vehicle driving on the road. I used a combination of center lane driving, 
recovering from the left and right sides of the road, and some turning corner data. And the data are fliped left to right to
 have some extra training data.

For details about how I created the training data, see the next section. 

### Model Architecture and Training Strategy

#### 1. Solution Design Approach

The overall strategy for deriving a model architecture was to use deep convolutional layer to capture the mark on road.

My first step was to use a convolution neural network model similar to the LeNet I thought this model might be appropriate because of the 
simplicity and performance on classification.

In order to gauge how well the model was working, I split my image and steering angle data into a training and validation set. 
I found that my first model had a low mean squared error on the training set but a high mean squared error on the validation set. 
This implied that the model was overfitting. 

To combat the overfitting, I modified the model so that it follows the nVidia network with more convolutional layer using strides.

Then I found the car has problem to recover from the edge of road. So I went back to collect more data about recovering from the side.

At the end of the process, the vehicle is able to drive autonomously around the track without leaving the road.

#### 2. Final Model Architecture

The final model architecture (model.py lines 130-140)  consisted of a convolution neural network with the following layers and layer sizes:

- normalization
- Convolution2D 24x5x5 with 2x2 strides, relu activation
- Convolution2D 36x5x5 with 2x2 strides, relu activation
- Convolution2D 48x5x5 with 2x2 strides, relu activation
- Convolution2D 64x3x3, relu activation
- Convolution2D 64x3x3, relu activation
- fully connected to 100 neurons
- fully connected to 50 neurons
- fully connected to 10 neurons
- fully connected to 1 neuron

Here is a visualization of the architecture 

![alt text][image1]

####3. Creation of the Training Set & Training Process

To capture good driving behavior, I first recorded two laps on track one using center lane driving. Here is an example image of center lane driving:

![alt text][image2]

I then recorded the vehicle recovering from the left side and right sides of the road back to center so that the vehicle would learn to .... These images show what a recovery looks like starting from ... :

![alt text][image3]

![alt text][image4]

![alt text][image5]

Then I repeated this process on track two in order to get more data points.

To augment the data sat, I also flipped images and angles thinking that this would create more training data. And steering left and right will be
 more balanced. For example, here is an image that has then been flipped:

![alt text][image6]

![alt text][image7]

After the collection process, I had 5144 number of data points. I then preprocessed this data by crop and top and bottom of the image, 
convert them into gray scale. Use the image from both left and right camaras with steering correction angle of 0.1.


I finally randomly shuffled the data set and put 20% of the data into a validation set. 

I used this training data for training the model. The validation set helped determine if the model was over or under fitting. 
I used an adam optimizer so that manually training the learning rate wasn't necessary.


### Final result (drives around track video)


Here's a [link to my video result](./run1.mp4)
