# FP.1 Match 3D Objects
- To match bounding boxes in previous and current frames, I used the vector of matched keypoints between previous and current frames,
and I iterate through each match and I retrive both matched keypoints from previous and current frame.
- Then I check, for each bounding box in previous frame if the matched keypoint in previous frame is within its ROI, I save the bounding box ID,
the same procedure applies for bounding boxes and matched keypoint in current frame.
- Then after saving both bounding boxes IDs, I build a map where each bounding box ID from the previous frame is mapped to each bounding box ID in the current frame,
along with the number of matched keypoints between both bounding boxes.
- After building the map, I iterate through each of bounding box ID in the previous frame saved in the map, which are the map keys, and map it to the bounding box ID
from the current frame which have the highest number of matched keypoints.
![alt text](https://i.ibb.co/yRNv3tz/1.png)


# FP.2 Compute Lidar-based TTC
- To compute the TTC based on Lidar data, I used the same solution from a previous lesson.
- Where from both previous and current frames, we get the point which have the smallest value in x coordinate.
- And to exclude outliers, we check if this point is outside the lane width, if yes, we won't check it.
![alt text](https://i.ibb.co/rf9Jpv0/2.png)


# FP.3 Associate Keypoint Correspondences with Bounding Boxes
- To associate keypoints to the bounding boxes enclosing them, I iterate through each of the matched keypoints.
- Then for each match, I check if the matched keypoint of the current frame is inside the ROI of the bounding box.
- If yes, then I add this keypoint to the list of keypoints enclosed by this bounding box, and also I add this match. 
![alt text](https://i.ibb.co/Tr740b6/3.png)

# FP.4 Compute Camera-based TTC
- To compute the TTC based on Camera data, I used the same solution from a previous lesson.
![alt text](https://i.ibb.co/RNYvdg3/4.png)


# Logging
- To log the TTC based on Lidar and Camera by using all the combinations of detectors and descriptors.
![alt text](https://i.ibb.co/vccpRmQ/5.png)
- And then I logged each metric to a csv file.
![alt text](https://i.ibb.co/m8wT0m2/6.png)

# FP.5 Performance Evaluation 1
- Here's two images where the computed TTC based on Lidar data is negative, which doesn't make sense.
- My argument on why the time is being negative is as follows, firstly the equation used to compute the TTC is based
on constant velocity model, where it assumes that the relative velocity between both ego and preceding car is always constant.
- Which in this situation, in some of the given frames, it seems like the ego car get more closer to the preceding car, but not with a constant amount,
which violates our assumption that the relative velocity is constant.
- Also in some frames, the preceding car accelerated and get slightly farther from the ego car.
- If we look at the values of the point in both previous and current frames in the x coordinate, for negative TTC, the value at the current frame is greater
than the previous one, which turns the TTC into a negative value.
- And that due to the assumption of the constant relative velocity, which is always positive and the ego car moves faster than the preceding car.
![alt text](https://i.ibb.co/XJmPV5f/Screenshot-from-2022-07-21-02-16-47.png)

![alt text](https://i.ibb.co/nrHLG3t/Screenshot-from-2022-07-21-02-17-45.png)


# FP.6 Performance Evaluation 2
- Please check csv files, ttc_lidar.csv, ttc_camera.csv .