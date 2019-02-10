# Traffic Sign Recognition


## Goals
* Load the data set
* Explore, summarize and visualize the data set
* Design, train and test a model architecture
* Use the model to make predictions on new images
* Analyze the softmax probabilities of the new images


[//]: # (Image References)

[image1]: ./output_img/training%20hist.png "Visualization"
[image2]: ./output_img/original.png "original"
[image3]: ./output_img/gray.png "gray"
[image4]: ./new_sign/1.jpg "Traffic Sign 1"
[image5]: ./new_sign/2.jpg "Traffic Sign 2"
[image6]: ./new_sign/3.jpg "Traffic Sign 3"
[image7]: ./new_sign/4.jpg "Traffic Sign 4"
[image8]: ./new_sign/5.jpg "Traffic Sign 5"

### Data Set Summary & Exploration

#### 1. Summary of the data set

I used the numpy library to calculate summary statistics of the traffic
signs data set:

* The size of training set is 34799
* The size of the validation set is 4410
* The size of test set is 12630
* The shape of a traffic sign image is (32, 32, 3)
* The number of unique classes/labels in the data set is 43

#### 2. Visualization of the dataset

Here is an exploratory visualization of the data set. It is a bar chart showing how is the distribution of the training data.

![alt text][image1]

### Design and Test a Model Architecture

#### 1. Preprocessed the image data

As a first step, I decided to convert the images to grayscale because this reduce the size of the data input.

Here is an example of a traffic sign image before and after grayscaling.

![alt text][image2] 
![alt text][image3]

As a last step, I normalized the image data because this gives 0 mean and equal
variance, easier for the optimizer to do the work.


#### 2. Final model architecture

My final model consisted of the following layers:

| Layer         		|     Description	        					| 
|:---------------------:|:---------------------------------------------:| 
| Input         		| 32x32x1 gray image   							| 
| Convolution 3x3     	| 1x1 stride, same padding, outputs 32x32x16 	|
| RELU					|												|
| Convolution 3x3     	| 1x1 stride, same padding, outputs 32x32x16 	|
| RELU					|												|
| Max pooling	      	| 2x2 stride,  outputs 16x16x16 				|
| Drop out  	      	| 50%                            				|
| Convolution 3x3     	| 1x1 stride, same padding, outputs 32x32x32 	|
| RELU					|												|
| Convolution 3x3     	| 1x1 stride, same padding, outputs 32x32x32 	|
| RELU					|												|
| Max pooling	      	| 2x2 stride,  outputs 16x16x64 				|
| Drop out  	      	| 50%                            				|
| Fully connected		| 2048x512    									|
| Fully connected		| 512x128         								|
| Fully connected		| 128x43         								|
| Softmax				| etc.        									|

 


#### 3. Train model

To train the model, I used an AdamOptimizer

| parameter         	|     setting	        					| 
|:---------------------:|:---------------------------------------------:| 
| batch size         		| 256 							| 
| number of epochs   	| 22 	|
| learning rate		| start at 0.0015, decrease when validation error decrease by 10%|


#### 4. Approach taken
My final model results were:
* training set accuracy of 98.4%
* validation set accuracy of 96.3%
* test set accuracy of 94.9%


* First architecture that was tried and why was it chosen?
    * RGB image with 2 convolution layers without dropout
    
* Some problems with the initial architecture?
    * RGB image takes far more computation to get trained
    * 2 convolution layers is not enough
    * without dropout, it easily overfit
    
* Architecture adjusted
    * use gray image reduce the computation power required
    * add more convolution layers with smaller patch size
    * add dropout to reduce overfit 
* Parameters tuned
    * the depth of the convolution is increased from 16 to 32
* Important design choices
    * reduce the patch size from 5 to 3, and increase the convolution layers.
    This increases the nonlinearity of the model, but with little cost on the computation.


### Test a Model on New Images

#### 1. Five German traffic signs found on the web


![alt text][image4]

This image is not square, which may make it difficult to classify after resize to input size.

![alt text][image5]

This sign has extra words below. 
 
![alt text][image6] 

This one has none square shape and background noise

![alt text][image7] 

This one has has none square shape, big size and noise on the sign

![alt text][image8]

This one has background noise

#### 2. Model's predictions

Here are the results of the prediction:

| Image			        |     Prediction	        					| 
|:---------------------:|:---------------------------------------------:| 
| 3 Speed limit (60km/h)		| 3 Speed limit (60km/h)							|
| 28 Children crossing    | 28 Children crossing  			 				|
| 14 Stop      		| 14 Stop      							| 
| 25 Road work 		| 18 General caution    							|
| 18 General caution    | 18 General caution    									|



The model was able to correctly guess 4 of the 5 traffic signs, which gives an accuracy of 80%.
The accuracy on the captured images is 80% while it was 94% on the testing set thus It seems the model is overfitting. The wrong classified image has the sign one the side of image and corner croped after preprocessing. So more training data on this kind of offset sign should help the overfitting.

#### 3. Softmax probabilities for each prediction.

The code for making predictions on my final model is located in the 18th cell of the Ipython notebook.

| Probability         	|     Prediction	        					| 
|:---------------------:|:---------------------------------------------:| 
| .86		| 3 Speed limit (60km/h)							|
| .99    | 28 Children crossing  			 				|
| .99     		| 14 Stop      							| 
| .70	| 18 General caution    							|
| .99   | 18 General caution    									|


the model is pretty sure about all of the results, although one of the result is wrong, which has lower probability 70%.

