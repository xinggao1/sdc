# **Finding Lane Lines on the Road** 
---

**Finding Lane Lines on the Road**

The goals / steps of this project are the following:
* Make a pipeline that finds lane lines on the road
* Reflect on your work in a written report


[//]: # (Image References)

[image1]: ./test_images_output/masked.png "m1"
[image2]: ./test_images_output/masked2.png "m2"
[image3]: ./test_images_output/masked3.png "m3"
[image4]: ./test_images_output/out.png "out"

---

### Reflection

### 1. Describe your pipeline. As part of the description, explain how you modified the draw_lines() function.

My pipeline consisted of 5 steps. 

    1. use gaussian blur to blur lightly
    2. take only the bottom triangle region_of_interest for further processing
        ![alt text][image1]
    
    3. calculate the edges and remove the borders from previous step
        ![alt text][image2]
    
    4. find line sections through cv2.HoughLinesP
        ![alt text][image3]
    
    5. sort all the line sections into different bins based on their angle and offset
    6. find the 2 bins with most of line secitons count by length, and output the final 2 lines through the weighted average. 
        ![alt text][image4]


### 2. Identify potential shortcomings with your current pipeline


    One potential shortcoming would be what would happen when the lane is turning, the lane line is not straight.
    Another would be when the road is dirty, the lane line has low contrast. 

### 3. Suggest possible improvements to your pipeline

A possible improvement would be to use the color character to improve, since the lane color are only either white or yellow.

