#**Traffic Sign Recognition** 

---

**Build a Traffic Sign Recognition Project**

The goals / steps of this project are the following:
* Load the data set (see below for links to the project data set)
* Explore, summarize and visualize the data set
* Design, train and test a model architecture
* Use the model to make predictions on new images
* Analyze the softmax probabilities of the new images
* Summarize the results with a written report


[//]: # (Image References)

[image1]: ./output_img/training%20hist.png "Visualization"
[image2]: ./output_img/original.png "original"
[image3]: ./output_img/gray.png "gray"
[image4]: ./new_sign/1.jpg "Traffic Sign 1"
[image5]: ./new_sign/2.jpg "Traffic Sign 2"
[image6]: ./new_sign/3.jpg "Traffic Sign 3"
[image7]: ./new_sign/4.jpg "Traffic Sign 4"
[image8]: ./new_sign/5.jpg "Traffic Sign 5"

---
###Writeup / README

####1. Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf. You can use this template as a guide for writing the report. The submission includes the project code.

You're reading it! and here is a link to my [project code](https://github.com/udacity/CarND-Traffic-Sign-Classifier-Project/blob/master/Traffic_Sign_Classifier.ipynb)

###Data Set Summary & Exploration

####1. Provide a basic summary of the data set. In the code, the analysis should be done using python, numpy and/or pandas methods rather than hardcoding results manually.

I used the numpy library to calculate summary statistics of the traffic
signs data set:

* The size of training set is 34799
* The size of the validation set is 4410
* The size of test set is 12630
* The shape of a traffic sign image is (32, 32, 3)
* The number of unique classes/labels in the data set is 43

####2. Include an exploratory visualization of the dataset.

Here is an exploratory visualization of the data set. It is a bar chart showing how is the distribution of the training data.

![alt text][image1]

###Design and Test a Model Architecture

####1. Describe how you preprocessed the image data. What techniques were chosen and why did you choose these techniques? Consider including images showing the output of each preprocessing technique. Pre-processing refers to techniques such as converting to grayscale, normalization, etc. (OPTIONAL: As described in the "Stand Out Suggestions" part of the rubric, if you generated additional data for training, describe why you decided to generate additional data, how you generated the data, and provide example images of the additional data. Then describe the characteristics of the augmented training set like number of images in the set, number of images for each class, etc.)

As a first step, I decided to convert the images to grayscale because this reduce the size of the data input.

Here is an example of a traffic sign image before and after grayscaling.

![alt text][image2] 
![alt text][image3]

As a last step, I normalized the image data because this gives 0 mean and equal variance, easier for the optimizer to do the work.


####2. Describe what your final model architecture looks like including model type, layers, layer sizes, connectivity, etc.) Consider including a diagram and/or table describing the final model.

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

 


####3. Describe how you trained your model. The discussion can include the type of optimizer, the batch size, number of epochs and any hyperparameters such as learning rate.

To train the model, I used an AdamOptimizer

| parameter         	|     setting	        					| 
|:---------------------:|:---------------------------------------------:| 
| batch size         		| 256 							| 
| number of epochs   	| 22 	|
| learning rate		| start at 0.0015, decrease when validation error decrease by 10%|


####4. Describe the approach taken 
My final model results were:
* training set accuracy of 98.4%
* validation set accuracy of 96.3%
* test set accuracy of 94.9%

If an iterative approach was chosen:
* What was the first architecture that was tried and why was it chosen?
    * RGB image with 2 convolution layers without dropout
    
* What were some problems with the initial architecture?
    * RGB image takes far more computation to get trained
    * 2 convolution layers is not enough
    * without dropout, it easily overfit
    
* How was the architecture adjusted and why was it adjusted? Typical adjustments could include choosing a different model architecture, adding or taking away layers (pooling, dropout, convolution, etc), using an activation function or changing the activation function. One common justification for adjusting an architecture would be due to overfitting or underfitting. A high accuracy on the training set but low accuracy on the validation set indicates over fitting; a low accuracy on both sets indicates under fitting.
    * use gray image reduce the computation power required
    * add more convolution layers with smaller patch size
    * add dropout to reduce overfit 
* Which parameters were tuned? How were they adjusted and why?
    * the depth of the convolution is increased from 16 to 32
* What are some of the important design choices and why were they chosen? For example, why might a convolution layer work well with this problem? How might a dropout layer help with creating a successful model?
    * reduce the patch size from 5 to 3, and increase the convolution layers. This increases the nonlinearity of the model, but with little cost on the computation.


###Test a Model on New Images

####1. Choose five German traffic signs found on the web and provide them in the report. For each image, discuss what quality or qualities might be difficult to classify.

Here are five German traffic signs that I found on the web:

![alt text][image4]
![alt text][image5] ![alt text][image6] 
![alt text][image7] ![alt text][image8]


####2. Discuss the model's predictions on these new traffic signs and compare the results to predicting on the test set. At a minimum, discuss what the predictions were, the accuracy on these new predictions, and compare the accuracy to the accuracy on the test set (OPTIONAL: Discuss the results in more detail as described in the "Stand Out Suggestions" part of the rubric).

Here are the results of the prediction:

| Image			        |     Prediction	        					| 
|:---------------------:|:---------------------------------------------:| 
| 3 Speed limit (60km/h)		| 3 Speed limit (60km/h)							|
| 28 Children crossing    | 28 Children crossing  			 				|
| 14 Stop      		| 14 Stop      							| 
| 25 Road work 		| 18 General caution    							|
| 18 General caution    | 18 General caution    									|



The model was able to correctly guess 4 of the 5 traffic signs, which gives an accuracy of 80%. 

####3. Describe how certain the model is when predicting on each of the five new images by looking at the softmax probabilities for each prediction. Provide the top 5 softmax probabilities for each image along with the sign type of each probability. (OPTIONAL: as described in the "Stand Out Suggestions" part of the rubric, visualizations can also be provided such as bar charts)

The code for making predictions on my final model is located in the 18th cell of the Ipython notebook.

| Probability         	|     Prediction	        					| 
|:---------------------:|:---------------------------------------------:| 
| .86		| 3 Speed limit (60km/h)							|
| .99    | 28 Children crossing  			 				|
| .99     		| 14 Stop      							| 
| .70	| 18 General caution    							|
| .99   | 18 General caution    									|


the model is pretty sure about all of the results, although one of the result is wrong, which has lower probability 70%.

